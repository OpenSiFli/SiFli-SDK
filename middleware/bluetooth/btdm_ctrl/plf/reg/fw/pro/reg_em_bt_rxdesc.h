#ifndef _REG_EM_BT_RXDESC_H_
#define _REG_EM_BT_RXDESC_H_

#include <stdint.h>
#include "_reg_em_bt_rxdesc.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BT_RXDESC_COUNT 8

#define REG_EM_BT_RXDESC_DECODING_MASK 0x0000000F

#define REG_EM_BT_RXDESC_ADDR_GET(idx) (EM_BT_RXDESC_OFFSET + (idx) * REG_EM_BT_RXDESC_SIZE)

/**
 * @brief RXPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               RXDONE   0
 *  13:00              NEXTPTR   0x0
 * </pre>
 */
#define EM_BT_RXPTR_ADDR   (0x204F0000 + EM_BT_RXDESC_OFFSET)
#define EM_BT_RXPTR_INDEX  0x00000000
#define EM_BT_RXPTR_RESET  0x00000000

__INLINE uint16_t em_bt_rxptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
}

__INLINE void em_bt_rxptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, value);
}

// field definitions
#define EM_BT_RXDONE_BIT     ((uint16_t)0x00008000)
#define EM_BT_RXDONE_POS     15
#define EM_BT_NEXTPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BT_NEXTPTR_LSB    0
#define EM_BT_NEXTPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BT_RXDONE_RST     0x0
#define EM_BT_NEXTPTR_RST    0x0

__INLINE void em_bt_rxptr_pack(int elt_idx, uint8_t rxdone, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BT_WR(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, ((uint16_t)rxdone << 15) | ((uint16_t)nextptr << 0));
}

__INLINE void em_bt_rxptr_unpack(int elt_idx, uint8_t *rxdone, uint16_t *nextptr)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);

    *rxdone = (localVal & ((uint16_t)0x00008000)) >> 15;
    *nextptr = (localVal & ((uint16_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t em_bt_rxptr_rxdone_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_rxptr_rxdone_setf(int elt_idx, uint8_t rxdone)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, (EM_BT_RD(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxdone << 15));
}

__INLINE uint16_t em_bt_rxptr_nextptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00003FFF)) >> 0);
}

__INLINE void em_bt_rxptr_nextptr_setf(int elt_idx, uint16_t nextptr)
{
    ASSERT_ERR((((uint16_t)nextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BT_WR(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, (EM_BT_RD(EM_BT_RXPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE) & ~((uint16_t)0x00003FFF)) | ((uint16_t)nextptr << 0));
}

/**
 * @brief RXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:11            RXLINKLBL   0x0
 *     08            RXEIRSTAT   0
 *     07           RXGUARDERR   0
 *     06              RXBADLT   0
 *     05             RXFECERR   0
 *     04             RXSEQERR   0
 *     03             RXMICERR   0
 *     02             RXCRCERR   0
 *     01             RXHECERR   0
 *     00            RXSYNCERR   0
 * </pre>
 */
#define EM_BT_RXSTAT_ADDR   (0x204F0002 + EM_BT_RXDESC_OFFSET)
#define EM_BT_RXSTAT_INDEX  0x00000001
#define EM_BT_RXSTAT_RESET  0x00000000

__INLINE uint16_t em_bt_rxstat_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
}

__INLINE void em_bt_rxstat_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, value);
}

// field definitions
#define EM_BT_RXLINKLBL_MASK    ((uint16_t)0x0000F800)
#define EM_BT_RXLINKLBL_LSB     11
#define EM_BT_RXLINKLBL_WIDTH   ((uint16_t)0x00000005)
#define EM_BT_RXEIRSTAT_BIT     ((uint16_t)0x00000100)
#define EM_BT_RXEIRSTAT_POS     8
#define EM_BT_RXGUARDERR_BIT    ((uint16_t)0x00000080)
#define EM_BT_RXGUARDERR_POS    7
#define EM_BT_RXBADLT_BIT       ((uint16_t)0x00000040)
#define EM_BT_RXBADLT_POS       6
#define EM_BT_RXFECERR_BIT      ((uint16_t)0x00000020)
#define EM_BT_RXFECERR_POS      5
#define EM_BT_RXSEQERR_BIT      ((uint16_t)0x00000010)
#define EM_BT_RXSEQERR_POS      4
#define EM_BT_RXMICERR_BIT      ((uint16_t)0x00000008)
#define EM_BT_RXMICERR_POS      3
#define EM_BT_RXCRCERR_BIT      ((uint16_t)0x00000004)
#define EM_BT_RXCRCERR_POS      2
#define EM_BT_RXHECERR_BIT      ((uint16_t)0x00000002)
#define EM_BT_RXHECERR_POS      1
#define EM_BT_RXSYNCERR_BIT     ((uint16_t)0x00000001)
#define EM_BT_RXSYNCERR_POS     0

#define EM_BT_RXLINKLBL_RST     0x0
#define EM_BT_RXEIRSTAT_RST     0x0
#define EM_BT_RXGUARDERR_RST    0x0
#define EM_BT_RXBADLT_RST       0x0
#define EM_BT_RXFECERR_RST      0x0
#define EM_BT_RXSEQERR_RST      0x0
#define EM_BT_RXMICERR_RST      0x0
#define EM_BT_RXCRCERR_RST      0x0
#define EM_BT_RXHECERR_RST      0x0
#define EM_BT_RXSYNCERR_RST     0x0

__INLINE void em_bt_rxstat_unpack(int elt_idx, uint8_t *rxlinklbl, uint8_t *rxeirstat, uint8_t *rxguarderr, uint8_t *rxbadlt, uint8_t *rxfecerr, uint8_t *rxseqerr, uint8_t *rxmicerr, uint8_t *rxcrcerr, uint8_t *rxhecerr, uint8_t *rxsyncerr)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);

    *rxlinklbl = (localVal & ((uint16_t)0x0000F800)) >> 11;
    *rxeirstat = (localVal & ((uint16_t)0x00000100)) >> 8;
    *rxguarderr = (localVal & ((uint16_t)0x00000080)) >> 7;
    *rxbadlt = (localVal & ((uint16_t)0x00000040)) >> 6;
    *rxfecerr = (localVal & ((uint16_t)0x00000020)) >> 5;
    *rxseqerr = (localVal & ((uint16_t)0x00000010)) >> 4;
    *rxmicerr = (localVal & ((uint16_t)0x00000008)) >> 3;
    *rxcrcerr = (localVal & ((uint16_t)0x00000004)) >> 2;
    *rxhecerr = (localVal & ((uint16_t)0x00000002)) >> 1;
    *rxsyncerr = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_bt_rxstat_rxlinklbl_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x0000F800)) >> 11);
}

__INLINE uint8_t em_bt_rxstat_rxeirstat_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE uint8_t em_bt_rxstat_rxguarderr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE uint8_t em_bt_rxstat_rxbadlt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE uint8_t em_bt_rxstat_rxfecerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE uint8_t em_bt_rxstat_rxseqerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE uint8_t em_bt_rxstat_rxmicerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE uint8_t em_bt_rxstat_rxcrcerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE uint8_t em_bt_rxstat_rxhecerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE uint8_t em_bt_rxstat_rxsyncerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXSTAT_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

/**
 * @brief RXHEADER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     09               RXSEQN   0
 *     08               RXARQN   0
 *     07               RXFLOW   0
 *  06:03               RXTYPE   0x0
 *  02:00             RXLTADDR   0x0
 * </pre>
 */
#define EM_BT_RXHEADER_ADDR   (0x204F0004 + EM_BT_RXDESC_OFFSET)
#define EM_BT_RXHEADER_INDEX  0x00000002
#define EM_BT_RXHEADER_RESET  0x00000000

__INLINE uint16_t em_bt_rxheader_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
}

__INLINE void em_bt_rxheader_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, value);
}

// field definitions
#define EM_BT_RXSEQN_BIT      ((uint16_t)0x00000200)
#define EM_BT_RXSEQN_POS      9
#define EM_BT_RXARQN_BIT      ((uint16_t)0x00000100)
#define EM_BT_RXARQN_POS      8
#define EM_BT_RXFLOW_BIT      ((uint16_t)0x00000080)
#define EM_BT_RXFLOW_POS      7
#define EM_BT_RXTYPE_MASK     ((uint16_t)0x00000078)
#define EM_BT_RXTYPE_LSB      3
#define EM_BT_RXTYPE_WIDTH    ((uint16_t)0x00000004)
#define EM_BT_RXLTADDR_MASK   ((uint16_t)0x00000007)
#define EM_BT_RXLTADDR_LSB    0
#define EM_BT_RXLTADDR_WIDTH  ((uint16_t)0x00000003)

#define EM_BT_RXSEQN_RST      0x0
#define EM_BT_RXARQN_RST      0x0
#define EM_BT_RXFLOW_RST      0x0
#define EM_BT_RXTYPE_RST      0x0
#define EM_BT_RXLTADDR_RST    0x0

__INLINE void em_bt_rxheader_unpack(int elt_idx, uint8_t *rxseqn, uint8_t *rxarqn, uint8_t *rxflow, uint8_t *rxtype, uint8_t *rxltaddr)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);

    *rxseqn = (localVal & ((uint16_t)0x00000200)) >> 9;
    *rxarqn = (localVal & ((uint16_t)0x00000100)) >> 8;
    *rxflow = (localVal & ((uint16_t)0x00000080)) >> 7;
    *rxtype = (localVal & ((uint16_t)0x00000078)) >> 3;
    *rxltaddr = (localVal & ((uint16_t)0x00000007)) >> 0;
}

__INLINE uint8_t em_bt_rxheader_rxseqn_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE uint8_t em_bt_rxheader_rxarqn_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE uint8_t em_bt_rxheader_rxflow_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE uint8_t em_bt_rxheader_rxtype_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000078)) >> 3);
}

__INLINE uint8_t em_bt_rxheader_rxltaddr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000007)) >> 0);
}

/**
 * @brief RXPHEADER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:13              RXUNDEF   0x0
 *  12:03             RXLENGTH   0x0
 *     02              RXPFLOW   0
 *  01:00               RXLLID   0x0
 * </pre>
 */
#define EM_BT_RXPHEADER_ADDR   (0x204F0006 + EM_BT_RXDESC_OFFSET)
#define EM_BT_RXPHEADER_INDEX  0x00000003
#define EM_BT_RXPHEADER_RESET  0x00000000

__INLINE uint16_t em_bt_rxpheader_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXPHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
}

__INLINE void em_bt_rxpheader_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXPHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, value);
}

// field definitions
#define EM_BT_RXUNDEF_MASK    ((uint16_t)0x0000E000)
#define EM_BT_RXUNDEF_LSB     13
#define EM_BT_RXUNDEF_WIDTH   ((uint16_t)0x00000003)
#define EM_BT_RXLENGTH_MASK   ((uint16_t)0x00001FF8)
#define EM_BT_RXLENGTH_LSB    3
#define EM_BT_RXLENGTH_WIDTH  ((uint16_t)0x0000000A)
#define EM_BT_RXPFLOW_BIT     ((uint16_t)0x00000004)
#define EM_BT_RXPFLOW_POS     2
#define EM_BT_RXLLID_MASK     ((uint16_t)0x00000003)
#define EM_BT_RXLLID_LSB      0
#define EM_BT_RXLLID_WIDTH    ((uint16_t)0x00000002)

#define EM_BT_RXUNDEF_RST     0x0
#define EM_BT_RXLENGTH_RST    0x0
#define EM_BT_RXPFLOW_RST     0x0
#define EM_BT_RXLLID_RST      0x0

__INLINE void em_bt_rxpheader_unpack(int elt_idx, uint8_t *rxundef, uint16_t *rxlength, uint8_t *rxpflow, uint8_t *rxllid)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);

    *rxundef = (localVal & ((uint16_t)0x0000E000)) >> 13;
    *rxlength = (localVal & ((uint16_t)0x00001FF8)) >> 3;
    *rxpflow = (localVal & ((uint16_t)0x00000004)) >> 2;
    *rxllid = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_bt_rxpheader_rxundef_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x0000E000)) >> 13);
}

__INLINE uint16_t em_bt_rxpheader_rxlength_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00001FF8)) >> 3);
}

__INLINE uint8_t em_bt_rxpheader_rxpflow_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE uint8_t em_bt_rxpheader_rxllid_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXPHEADER_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

/**
 * @brief RXCHASS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15             IS_AUDIO   0
 *  14:08            RXCHANNEL   0x0
 *  07:00               RXRSSI   0x0
 * </pre>
 */
#define EM_BT_RXCHASS_ADDR   (0x204F0008 + EM_BT_RXDESC_OFFSET)
#define EM_BT_RXCHASS_INDEX  0x00000004
#define EM_BT_RXCHASS_RESET  0x00000000

__INLINE uint16_t em_bt_rxchass_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXCHASS_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
}

__INLINE void em_bt_rxchass_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXCHASS_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, value);
}

// field definitions
#define EM_BT_IS_AUDIO_BIT     ((uint16_t)0x00008000)
#define EM_BT_IS_AUDIO_POS     15
#define EM_BT_RXCHANNEL_MASK   ((uint16_t)0x00007F00)
#define EM_BT_RXCHANNEL_LSB    8
#define EM_BT_RXCHANNEL_WIDTH  ((uint16_t)0x00000007)
#define EM_BT_RXRSSI_MASK      ((uint16_t)0x000000FF)
#define EM_BT_RXRSSI_LSB       0
#define EM_BT_RXRSSI_WIDTH     ((uint16_t)0x00000008)

#define EM_BT_IS_AUDIO_RST     0x0
#define EM_BT_RXCHANNEL_RST    0x0
#define EM_BT_RXRSSI_RST       0x0

__INLINE void em_bt_rxchass_unpack(int elt_idx, uint8_t *isaudio, uint8_t *rxchannel, uint8_t *rxrssi)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCHASS_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);

    *isaudio = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxchannel = (localVal & ((uint16_t)0x00007F00)) >> 8;
    *rxrssi = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_bt_rxchass_is_audio_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCHASS_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE uint8_t em_bt_rxchass_rxchannel_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCHASS_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x00007F00)) >> 8);
}

__INLINE uint8_t em_bt_rxchass_rxrssi_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCHASS_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

/**
 * @brief RXACLBUFPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          RXACLBUFPTR   0x0
 * </pre>
 */
#define EM_BT_RXACLBUFPTR_ADDR   (0x204F000A + EM_BT_RXDESC_OFFSET)
#define EM_BT_RXACLBUFPTR_INDEX  0x00000005
#define EM_BT_RXACLBUFPTR_RESET  0x00000000

__INLINE uint16_t em_bt_rxaclbufptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
}

__INLINE void em_bt_rxaclbufptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, value);
}

// field definitions
#define EM_BT_RXACLBUFPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_RXACLBUFPTR_LSB    0
#define EM_BT_RXACLBUFPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_RXACLBUFPTR_RST    0x0

__INLINE uint16_t em_bt_rxaclbufptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_rxaclbufptr_setf(int elt_idx, uint16_t rxaclbufptr)
{
    ASSERT_ERR((((uint16_t)rxaclbufptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_RXACLBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, (uint16_t)rxaclbufptr << 0);
}

/**
 * @brief RXLMBUFPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00           RXLMBUFPTR   0x0
 * </pre>
 */
#define EM_BT_RXLMBUFPTR_ADDR   (0x204F000C + EM_BT_RXDESC_OFFSET)
#define EM_BT_RXLMBUFPTR_INDEX  0x00000006
#define EM_BT_RXLMBUFPTR_RESET  0x00000000

__INLINE uint16_t em_bt_rxlmbufptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
}

__INLINE void em_bt_rxlmbufptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, value);
}

// field definitions
#define EM_BT_RXLMBUFPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_RXLMBUFPTR_LSB    0
#define EM_BT_RXLMBUFPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_RXLMBUFPTR_RST    0x0

__INLINE uint16_t em_bt_rxlmbufptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_rxlmbufptr_setf(int elt_idx, uint16_t rxlmbufptr)
{
    ASSERT_ERR((((uint16_t)rxlmbufptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_RXLMBUFPTR_ADDR + elt_idx * REG_EM_BT_RXDESC_SIZE, (uint16_t)rxlmbufptr << 0);
}


#endif // _REG_EM_BT_RXDESC_H_

