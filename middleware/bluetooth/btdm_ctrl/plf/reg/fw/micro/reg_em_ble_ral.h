#ifndef _REG_EM_BLE_RAL_H_
#define _REG_EM_BLE_RAL_H_

#include <stdint.h>
#include "_reg_em_ble_ral.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_RAL_COUNT 28

#define REG_EM_BLE_RAL_DECODING_MASK 0x0000003F

#define REG_EM_BLE_RAL_ADDR_GET(idx) (EM_BLE_RAL_OFFSET + (idx) * REG_EM_BLE_RAL_SIZE)

/**
 * @brief RAL_INFO register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15          ENTRY_VALID   0
 *     14            CONNECTED   0
 *     13            IN_WHLIST   0
 *     12       IN_PERADV_LIST   0
 *     11                  PEF   0
 *     07      LOCAL_RPA_VALID   0
 *     06      LOCAL_RPA_RENEW   0
 *     05      LOCAL_IRK_VALID   0
 *     03       PEER_RPA_VALID   0
 *     02       PEER_RPA_RENEW   0
 *     01       PEER_IRK_VALID   0
 *     00         PEER_ID_TYPE   0
 * </pre>
 */
#define EM_BLE_RAL_INFO_ADDR   (0x20408000 + EM_BLE_RAL_OFFSET)
#define EM_BLE_RAL_INFO_INDEX  0x00000000
#define EM_BLE_RAL_INFO_RESET  0x00000000

__INLINE uint16_t em_ble_ral_info_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
}

__INLINE void em_ble_ral_info_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, value);
}

// field definitions
#define EM_BLE_ENTRY_VALID_BIT        ((uint16_t)0x00008000)
#define EM_BLE_ENTRY_VALID_POS        15
#define EM_BLE_CONNECTED_BIT          ((uint16_t)0x00004000)
#define EM_BLE_CONNECTED_POS          14
#define EM_BLE_IN_WHLIST_BIT          ((uint16_t)0x00002000)
#define EM_BLE_IN_WHLIST_POS          13
#define EM_BLE_IN_PERADV_LIST_BIT     ((uint16_t)0x00001000)
#define EM_BLE_IN_PERADV_LIST_POS     12
#define EM_BLE_PEF_BIT                ((uint16_t)0x00000800)
#define EM_BLE_PEF_POS                11
#define EM_BLE_LOCAL_RPA_VALID_BIT    ((uint16_t)0x00000080)
#define EM_BLE_LOCAL_RPA_VALID_POS    7
#define EM_BLE_LOCAL_RPA_RENEW_BIT    ((uint16_t)0x00000040)
#define EM_BLE_LOCAL_RPA_RENEW_POS    6
#define EM_BLE_LOCAL_IRK_VALID_BIT    ((uint16_t)0x00000020)
#define EM_BLE_LOCAL_IRK_VALID_POS    5
#define EM_BLE_PEER_RPA_VALID_BIT     ((uint16_t)0x00000008)
#define EM_BLE_PEER_RPA_VALID_POS     3
#define EM_BLE_PEER_RPA_RENEW_BIT     ((uint16_t)0x00000004)
#define EM_BLE_PEER_RPA_RENEW_POS     2
#define EM_BLE_PEER_IRK_VALID_BIT     ((uint16_t)0x00000002)
#define EM_BLE_PEER_IRK_VALID_POS     1
#define EM_BLE_PEER_ID_TYPE_BIT       ((uint16_t)0x00000001)
#define EM_BLE_PEER_ID_TYPE_POS       0

#define EM_BLE_ENTRY_VALID_RST        0x0
#define EM_BLE_CONNECTED_RST          0x0
#define EM_BLE_IN_WHLIST_RST          0x0
#define EM_BLE_IN_PERADV_LIST_RST     0x0
#define EM_BLE_PEF_RST                0x0
#define EM_BLE_LOCAL_RPA_VALID_RST    0x0
#define EM_BLE_LOCAL_RPA_RENEW_RST    0x0
#define EM_BLE_LOCAL_IRK_VALID_RST    0x0
#define EM_BLE_PEER_RPA_VALID_RST     0x0
#define EM_BLE_PEER_RPA_RENEW_RST     0x0
#define EM_BLE_PEER_IRK_VALID_RST     0x0
#define EM_BLE_PEER_ID_TYPE_RST       0x0

__INLINE void em_ble_ral_info_pack(int elt_idx, uint8_t entryvalid, uint8_t connected, uint8_t inwhlist, uint8_t inperadvlist, uint8_t pef, uint8_t localrpavalid, uint8_t localrparenew, uint8_t localirkvalid, uint8_t peerrpavalid, uint8_t peerrparenew, uint8_t peerirkvalid, uint8_t peeridtype)
{
    ASSERT_ERR((((uint16_t)entryvalid << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)connected << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)inwhlist << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)inperadvlist << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)pef << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)localrpavalid << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)localrparenew << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)localirkvalid << 5) & ~((uint16_t)0x00000020)) == 0);
    ASSERT_ERR((((uint16_t)peerrpavalid << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)peerrparenew << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)peerirkvalid << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)peeridtype << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, ((uint16_t)entryvalid << 15) | ((uint16_t)connected << 14) | ((uint16_t)inwhlist << 13) | ((uint16_t)inperadvlist << 12) | ((uint16_t)pef << 11) | ((uint16_t)localrpavalid << 7) | ((uint16_t)localrparenew << 6) | ((uint16_t)localirkvalid << 5) | ((uint16_t)peerrpavalid << 3) | ((uint16_t)peerrparenew << 2) | ((uint16_t)peerirkvalid << 1) | ((uint16_t)peeridtype << 0));
}

__INLINE void em_ble_ral_info_unpack(int elt_idx, uint8_t *entryvalid, uint8_t *connected, uint8_t *inwhlist, uint8_t *inperadvlist, uint8_t *pef, uint8_t *localrpavalid, uint8_t *localrparenew, uint8_t *localirkvalid, uint8_t *peerrpavalid, uint8_t *peerrparenew, uint8_t *peerirkvalid, uint8_t *peeridtype)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);

    *entryvalid = (localVal & ((uint16_t)0x00008000)) >> 15;
    *connected = (localVal & ((uint16_t)0x00004000)) >> 14;
    *inwhlist = (localVal & ((uint16_t)0x00002000)) >> 13;
    *inperadvlist = (localVal & ((uint16_t)0x00001000)) >> 12;
    *pef = (localVal & ((uint16_t)0x00000800)) >> 11;
    *localrpavalid = (localVal & ((uint16_t)0x00000080)) >> 7;
    *localrparenew = (localVal & ((uint16_t)0x00000040)) >> 6;
    *localirkvalid = (localVal & ((uint16_t)0x00000020)) >> 5;
    *peerrpavalid = (localVal & ((uint16_t)0x00000008)) >> 3;
    *peerrparenew = (localVal & ((uint16_t)0x00000004)) >> 2;
    *peerirkvalid = (localVal & ((uint16_t)0x00000002)) >> 1;
    *peeridtype = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_ral_info_entry_valid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_ral_info_entry_valid_setf(int elt_idx, uint8_t entryvalid)
{
    ASSERT_ERR((((uint16_t)entryvalid << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)entryvalid << 15));
}

__INLINE uint8_t em_ble_ral_info_connected_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_ble_ral_info_connected_setf(int elt_idx, uint8_t connected)
{
    ASSERT_ERR((((uint16_t)connected << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)connected << 14));
}

__INLINE uint8_t em_ble_ral_info_in_whlist_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_ble_ral_info_in_whlist_setf(int elt_idx, uint8_t inwhlist)
{
    ASSERT_ERR((((uint16_t)inwhlist << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)inwhlist << 13));
}

__INLINE uint8_t em_ble_ral_info_in_peradv_list_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_ble_ral_info_in_peradv_list_setf(int elt_idx, uint8_t inperadvlist)
{
    ASSERT_ERR((((uint16_t)inperadvlist << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)inperadvlist << 12));
}

__INLINE uint8_t em_ble_ral_info_pef_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_ble_ral_info_pef_setf(int elt_idx, uint8_t pef)
{
    ASSERT_ERR((((uint16_t)pef << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)pef << 11));
}

__INLINE uint8_t em_ble_ral_info_local_rpa_valid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_ble_ral_info_local_rpa_valid_setf(int elt_idx, uint8_t localrpavalid)
{
    ASSERT_ERR((((uint16_t)localrpavalid << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)localrpavalid << 7));
}

__INLINE uint8_t em_ble_ral_info_local_rpa_renew_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_ral_info_local_rpa_renew_setf(int elt_idx, uint8_t localrparenew)
{
    ASSERT_ERR((((uint16_t)localrparenew << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)localrparenew << 6));
}

__INLINE uint8_t em_ble_ral_info_local_irk_valid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE void em_ble_ral_info_local_irk_valid_setf(int elt_idx, uint8_t localirkvalid)
{
    ASSERT_ERR((((uint16_t)localirkvalid << 5) & ~((uint16_t)0x00000020)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000020)) | ((uint16_t)localirkvalid << 5));
}

__INLINE uint8_t em_ble_ral_info_peer_rpa_valid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_ral_info_peer_rpa_valid_setf(int elt_idx, uint8_t peerrpavalid)
{
    ASSERT_ERR((((uint16_t)peerrpavalid << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)peerrpavalid << 3));
}

__INLINE uint8_t em_ble_ral_info_peer_rpa_renew_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_ral_info_peer_rpa_renew_setf(int elt_idx, uint8_t peerrparenew)
{
    ASSERT_ERR((((uint16_t)peerrparenew << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)peerrparenew << 2));
}

__INLINE uint8_t em_ble_ral_info_peer_irk_valid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_ral_info_peer_irk_valid_setf(int elt_idx, uint8_t peerirkvalid)
{
    ASSERT_ERR((((uint16_t)peerirkvalid << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)peerirkvalid << 1));
}

__INLINE uint8_t em_ble_ral_info_peer_id_type_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_ral_info_peer_id_type_setf(int elt_idx, uint8_t peeridtype)
{
    ASSERT_ERR((((uint16_t)peeridtype << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (EM_BLE_RD(EM_BLE_RAL_INFO_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)peeridtype << 0));
}

/**
 * @brief RAL_PEER_SID register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         RAL_PEER_SID   0x0
 * </pre>
 */
#define EM_BLE_RAL_PEER_SID_ADDR   (0x20408002 + EM_BLE_RAL_OFFSET)
#define EM_BLE_RAL_PEER_SID_INDEX  0x00000001
#define EM_BLE_RAL_PEER_SID_RESET  0x00000000

__INLINE uint16_t em_ble_ral_peer_sid_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RAL_PEER_SID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
}

__INLINE void em_ble_ral_peer_sid_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RAL_PEER_SID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, value);
}

// field definitions
#define EM_BLE_RAL_PEER_SID_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RAL_PEER_SID_LSB    0
#define EM_BLE_RAL_PEER_SID_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RAL_PEER_SID_RST    0x0

__INLINE uint16_t em_ble_ral_peer_sid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_PEER_SID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_ral_peer_sid_setf(int elt_idx, uint16_t ralpeersid)
{
    ASSERT_ERR((((uint16_t)ralpeersid << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RAL_PEER_SID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE, (uint16_t)ralpeersid << 0);
}

/**
 * @brief RAL_PEER_IRK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             PEER_IRK   0x0
 * </pre>
 */
#define EM_BLE_RAL_PEER_IRK_ADDR   (0x20408004 + EM_BLE_RAL_OFFSET)
#define EM_BLE_RAL_PEER_IRK_INDEX  0x00000002
#define EM_BLE_RAL_PEER_IRK_RESET  0x00000000
#define EM_BLE_RAL_PEER_IRK_COUNT  8

__INLINE uint16_t em_ble_ral_peer_irk_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    return EM_BLE_RD(EM_BLE_RAL_PEER_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
}

__INLINE void em_ble_ral_peer_irk_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 7);
    EM_BLE_WR(EM_BLE_RAL_PEER_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_PEER_IRK_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_PEER_IRK_LSB    0
#define EM_BLE_PEER_IRK_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_PEER_IRK_RST    0x0

__INLINE uint16_t em_ble_ral_peer_irk_peer_irk_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_PEER_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_ral_peer_irk_peer_irk_setf(int elt_idx, int reg_idx, uint16_t peerirk)
{
    ASSERT_ERR(reg_idx <= 7);
    ASSERT_ERR((((uint16_t)peerirk << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RAL_PEER_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, (uint16_t)peerirk << 0);
}

/**
 * @brief RAL_PEER_RPA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             PEER_RPA   0x0
 * </pre>
 */
#define EM_BLE_RAL_PEER_RPA_ADDR   (0x20408014 + EM_BLE_RAL_OFFSET)
#define EM_BLE_RAL_PEER_RPA_INDEX  0x0000000A
#define EM_BLE_RAL_PEER_RPA_RESET  0x00000000
#define EM_BLE_RAL_PEER_RPA_COUNT  3

__INLINE uint16_t em_ble_ral_peer_rpa_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(EM_BLE_RAL_PEER_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
}

__INLINE void em_ble_ral_peer_rpa_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(EM_BLE_RAL_PEER_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_PEER_RPA_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_PEER_RPA_LSB    0
#define EM_BLE_PEER_RPA_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_PEER_RPA_RST    0x0

__INLINE uint16_t em_ble_ral_peer_rpa_peer_rpa_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_PEER_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_ral_peer_rpa_peer_rpa_setf(int elt_idx, int reg_idx, uint16_t peerrpa)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)peerrpa << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RAL_PEER_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, (uint16_t)peerrpa << 0);
}

/**
 * @brief RAL_PEER_ID register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              PEER_ID   0x0
 * </pre>
 */
#define EM_BLE_RAL_PEER_ID_ADDR   (0x2040801A + EM_BLE_RAL_OFFSET)
#define EM_BLE_RAL_PEER_ID_INDEX  0x0000000D
#define EM_BLE_RAL_PEER_ID_RESET  0x00000000
#define EM_BLE_RAL_PEER_ID_COUNT  3

__INLINE uint16_t em_ble_ral_peer_id_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(EM_BLE_RAL_PEER_ID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
}

__INLINE void em_ble_ral_peer_id_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(EM_BLE_RAL_PEER_ID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_PEER_ID_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_PEER_ID_LSB    0
#define EM_BLE_PEER_ID_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_PEER_ID_RST    0x0

__INLINE uint16_t em_ble_ral_peer_id_peer_id_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_PEER_ID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_ral_peer_id_peer_id_setf(int elt_idx, int reg_idx, uint16_t peerid)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)peerid << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RAL_PEER_ID_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, (uint16_t)peerid << 0);
}

/**
 * @brief RAL_LOCAL_IRK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            LOCAL_IRK   0x0
 * </pre>
 */
#define EM_BLE_RAL_LOCAL_IRK_ADDR   (0x20408020 + EM_BLE_RAL_OFFSET)
#define EM_BLE_RAL_LOCAL_IRK_INDEX  0x00000010
#define EM_BLE_RAL_LOCAL_IRK_RESET  0x00000000
#define EM_BLE_RAL_LOCAL_IRK_COUNT  8

__INLINE uint16_t em_ble_ral_local_irk_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    return EM_BLE_RD(EM_BLE_RAL_LOCAL_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
}

__INLINE void em_ble_ral_local_irk_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 7);
    EM_BLE_WR(EM_BLE_RAL_LOCAL_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_LOCAL_IRK_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_LOCAL_IRK_LSB    0
#define EM_BLE_LOCAL_IRK_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_LOCAL_IRK_RST    0x0

__INLINE uint16_t em_ble_ral_local_irk_local_irk_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_LOCAL_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_ral_local_irk_local_irk_setf(int elt_idx, int reg_idx, uint16_t localirk)
{
    ASSERT_ERR(reg_idx <= 7);
    ASSERT_ERR((((uint16_t)localirk << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RAL_LOCAL_IRK_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, (uint16_t)localirk << 0);
}

/**
 * @brief RAL_LOCAL_RPA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            LOCAL_RPA   0x0
 * </pre>
 */
#define EM_BLE_RAL_LOCAL_RPA_ADDR   (0x20408030 + EM_BLE_RAL_OFFSET)
#define EM_BLE_RAL_LOCAL_RPA_INDEX  0x00000018
#define EM_BLE_RAL_LOCAL_RPA_RESET  0x00000000
#define EM_BLE_RAL_LOCAL_RPA_COUNT  3

__INLINE uint16_t em_ble_ral_local_rpa_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(EM_BLE_RAL_LOCAL_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
}

__INLINE void em_ble_ral_local_rpa_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(EM_BLE_RAL_LOCAL_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_LOCAL_RPA_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_LOCAL_RPA_LSB    0
#define EM_BLE_LOCAL_RPA_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_LOCAL_RPA_RST    0x0

__INLINE uint16_t em_ble_ral_local_rpa_local_rpa_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RAL_LOCAL_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_ral_local_rpa_local_rpa_setf(int elt_idx, int reg_idx, uint16_t localrpa)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)localrpa << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RAL_LOCAL_RPA_ADDR + elt_idx * REG_EM_BLE_RAL_SIZE + reg_idx * 2, (uint16_t)localrpa << 0);
}


#endif // _REG_EM_BLE_RAL_H_

