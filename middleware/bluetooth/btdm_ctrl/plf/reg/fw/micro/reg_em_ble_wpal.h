#ifndef _REG_EM_BLE_WPAL_H_
#define _REG_EM_BLE_WPAL_H_

#include <stdint.h>
#include "_reg_em_ble_wpal.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_WPAL_COUNT 6

#define REG_EM_BLE_WPAL_DECODING_MASK 0x0000000F

#define REG_EM_BLE_WPAL_ADDR_GET(idx) (EM_BLE_WPAL_OFFSET + (idx) * REG_EM_BLE_WPAL_SIZE)

/**
 * @brief LIST_INFO register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15          ENTRY_VALID   0
 *     14               IDTYPE   0
 *     01                IN_WL   0
 *     00           IN_PERADVL   0
 * </pre>
 */
#define EM_BLE_LIST_INFO_ADDR   (0x20408000 + EM_BLE_WPAL_OFFSET)
#define EM_BLE_LIST_INFO_INDEX  0x00000000
#define EM_BLE_LIST_INFO_RESET  0x00000000

__INLINE uint16_t em_ble_list_info_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);
}

__INLINE void em_ble_list_info_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, value);
}

// field definitions
#define EM_BLE_ENTRY_VALID_BIT    ((uint16_t)0x00008000)
#define EM_BLE_ENTRY_VALID_POS    15
#define EM_BLE_IDTYPE_BIT         ((uint16_t)0x00004000)
#define EM_BLE_IDTYPE_POS         14
#define EM_BLE_IN_WL_BIT          ((uint16_t)0x00000002)
#define EM_BLE_IN_WL_POS          1
#define EM_BLE_IN_PERADVL_BIT     ((uint16_t)0x00000001)
#define EM_BLE_IN_PERADVL_POS     0

#define EM_BLE_ENTRY_VALID_RST    0x0
#define EM_BLE_IDTYPE_RST         0x0
#define EM_BLE_IN_WL_RST          0x0
#define EM_BLE_IN_PERADVL_RST     0x0

__INLINE void em_ble_list_info_pack(int elt_idx, uint8_t entryvalid, uint8_t idtype, uint8_t inwl, uint8_t inperadvl)
{
    ASSERT_ERR((((uint16_t)entryvalid << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)idtype << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)inwl << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)inperadvl << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, ((uint16_t)entryvalid << 15) | ((uint16_t)idtype << 14) | ((uint16_t)inwl << 1) | ((uint16_t)inperadvl << 0));
}

__INLINE void em_ble_list_info_unpack(int elt_idx, uint8_t *entryvalid, uint8_t *idtype, uint8_t *inwl, uint8_t *inperadvl)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);

    *entryvalid = (localVal & ((uint16_t)0x00008000)) >> 15;
    *idtype = (localVal & ((uint16_t)0x00004000)) >> 14;
    *inwl = (localVal & ((uint16_t)0x00000002)) >> 1;
    *inperadvl = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_list_info_entry_valid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_list_info_entry_valid_setf(int elt_idx, uint8_t entryvalid)
{
    ASSERT_ERR((((uint16_t)entryvalid << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, (EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)entryvalid << 15));
}

__INLINE uint8_t em_ble_list_info_idtype_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_ble_list_info_idtype_setf(int elt_idx, uint8_t idtype)
{
    ASSERT_ERR((((uint16_t)idtype << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BLE_WR(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, (EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)idtype << 14));
}

__INLINE uint8_t em_ble_list_info_in_wl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_list_info_in_wl_setf(int elt_idx, uint8_t inwl)
{
    ASSERT_ERR((((uint16_t)inwl << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, (EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)inwl << 1));
}

__INLINE uint8_t em_ble_list_info_in_peradvl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_list_info_in_peradvl_setf(int elt_idx, uint8_t inperadvl)
{
    ASSERT_ERR((((uint16_t)inperadvl << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, (EM_BLE_RD(EM_BLE_LIST_INFO_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)inperadvl << 0));
}

/**
 * @brief LIST_BDADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              LBDADDR   0x0
 * </pre>
 */
#define EM_BLE_LIST_BDADDR_ADDR   (0x20408002 + EM_BLE_WPAL_OFFSET)
#define EM_BLE_LIST_BDADDR_INDEX  0x00000001
#define EM_BLE_LIST_BDADDR_RESET  0x00000000
#define EM_BLE_LIST_BDADDR_COUNT  3

__INLINE uint16_t em_ble_list_bdaddr_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(EM_BLE_LIST_BDADDR_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE + reg_idx * 2);
}

__INLINE void em_ble_list_bdaddr_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(EM_BLE_LIST_BDADDR_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_LBDADDR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_LBDADDR_LSB    0
#define EM_BLE_LBDADDR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_LBDADDR_RST    0x0

__INLINE uint16_t em_ble_list_bdaddr_lbdaddr_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(EM_BLE_LIST_BDADDR_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_list_bdaddr_lbdaddr_setf(int elt_idx, int reg_idx, uint16_t lbdaddr)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)lbdaddr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_LIST_BDADDR_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE + reg_idx * 2, (uint16_t)lbdaddr << 0);
}

/**
 * @brief LIST_SID register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                 LSID   0x0
 * </pre>
 */
#define EM_BLE_LIST_SID_ADDR   (0x20408008 + EM_BLE_WPAL_OFFSET)
#define EM_BLE_LIST_SID_INDEX  0x00000004
#define EM_BLE_LIST_SID_RESET  0x00000000

__INLINE uint16_t em_ble_list_sid_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_LIST_SID_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);
}

__INLINE void em_ble_list_sid_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_LIST_SID_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, value);
}

// field definitions
#define EM_BLE_LSID_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_LSID_LSB    0
#define EM_BLE_LSID_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_LSID_RST    0x0

__INLINE uint16_t em_ble_list_sid_lsid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LIST_SID_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_list_sid_lsid_setf(int elt_idx, uint16_t lsid)
{
    ASSERT_ERR((((uint16_t)lsid << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_LIST_SID_ADDR + elt_idx * REG_EM_BLE_WPAL_SIZE, (uint16_t)lsid << 0);
}


#endif // _REG_EM_BLE_WPAL_H_

