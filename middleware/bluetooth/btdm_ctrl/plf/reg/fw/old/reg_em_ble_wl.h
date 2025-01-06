#ifndef _REG_EM_BLE_WL_H_
#define _REG_EM_BLE_WL_H_

#include <stdint.h>
#include "_reg_em_ble_wl.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_WL_COUNT 2

#define REG_EM_BLE_WL_DECODING_MASK 0x00000003

#define REG_EM_BLE_WL_ADDR_GET(idx) (EM_BLE_WL_OFFSET + (idx) * REG_EM_BLE_WL_SIZE)

/**
 * @brief WLIST_INFO register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15        WLENTRY_VALID   0
 *     14            WL_IDTYPE   0
 *     01              WL_RSVD   0
 *     00           IN_PERADVL   0
 * </pre>
 */
#define EM_BLE_WLIST_INFO_ADDR   (0x40110000 + EM_BLE_WL_OFFSET)
#define EM_BLE_WLIST_INFO_INDEX  0x00000000
#define EM_BLE_WLIST_INFO_RESET  0x00000000

__INLINE uint16_t em_ble_wlist_info_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE);
}

__INLINE void em_ble_wlist_info_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE, value);
}

// field definitions
#define EM_BLE_WLENTRY_VALID_BIT    ((uint16_t)0x00008000)
#define EM_BLE_WLENTRY_VALID_POS    15
#define EM_BLE_WL_IDTYPE_BIT        ((uint16_t)0x00004000)
#define EM_BLE_WL_IDTYPE_POS        14
#define EM_BLE_WL_RSVD_BIT          ((uint16_t)0x00000002)
#define EM_BLE_WL_RSVD_POS          1
#define EM_BLE_IN_PERADVL_BIT       ((uint16_t)0x00000001)
#define EM_BLE_IN_PERADVL_POS       0

#define EM_BLE_WLENTRY_VALID_RST    0x0
#define EM_BLE_WL_IDTYPE_RST        0x0
#define EM_BLE_WL_RSVD_RST          0x0
#define EM_BLE_IN_PERADVL_RST       0x0

__INLINE void em_ble_wlist_info_pack(int elt_idx, uint8_t wlentryvalid, uint8_t wlidtype, uint8_t wlrsvd, uint8_t inperadvl)
{
    ASSERT_ERR((((uint16_t)wlentryvalid << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)wlidtype << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)wlrsvd << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)inperadvl << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE, ((uint16_t)wlentryvalid << 15) | ((uint16_t)wlidtype << 14) | ((uint16_t)wlrsvd << 1) | ((uint16_t)inperadvl << 0));
}

__INLINE void em_ble_wlist_info_unpack(int elt_idx, uint8_t *wlentryvalid, uint8_t *wlidtype, uint8_t *wlrsvd, uint8_t *inperadvl)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE);

    *wlentryvalid = (localVal & ((uint16_t)0x00008000)) >> 15;
    *wlidtype = (localVal & ((uint16_t)0x00004000)) >> 14;
    *wlrsvd = (localVal & ((uint16_t)0x00000002)) >> 1;
    *inperadvl = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_wlist_info_wlentry_valid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_wlist_info_wlentry_valid_setf(int elt_idx, uint8_t wlentryvalid)
{
    ASSERT_ERR((((uint16_t)wlentryvalid << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE, (EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)wlentryvalid << 15));
}

__INLINE uint8_t em_ble_wlist_info_wl_idtype_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_ble_wlist_info_wl_idtype_setf(int elt_idx, uint8_t wlidtype)
{
    ASSERT_ERR((((uint16_t)wlidtype << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BLE_WR(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE, (EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)wlidtype << 14));
}

__INLINE uint8_t em_ble_wlist_info_wl_rsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_wlist_info_wl_rsvd_setf(int elt_idx, uint8_t wlrsvd)
{
    ASSERT_ERR((((uint16_t)wlrsvd << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE, (EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)wlrsvd << 1));
}

__INLINE uint8_t em_ble_wlist_info_in_peradvl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_wlist_info_in_peradvl_setf(int elt_idx, uint8_t inperadvl)
{
    ASSERT_ERR((((uint16_t)inperadvl << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE, (EM_BLE_RD(EM_BLE_WLIST_INFO_ADDR + elt_idx * REG_EM_BLE_WL_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)inperadvl << 0));
}

/**
 * @brief WL_BDADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             WLBDADDR   0x0
 * </pre>
 */
#define EM_BLE_WL_BDADDR_ADDR   (0x40110002 + EM_BLE_WL_OFFSET)
#define EM_BLE_WL_BDADDR_INDEX  0x00000001
#define EM_BLE_WL_BDADDR_RESET  0x00000000
#define EM_BLE_WL_BDADDR_COUNT  3

__INLINE uint16_t em_ble_wl_bdaddr_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(EM_BLE_WL_BDADDR_ADDR + elt_idx * REG_EM_BLE_WL_SIZE + reg_idx * 2);
}

__INLINE void em_ble_wl_bdaddr_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(EM_BLE_WL_BDADDR_ADDR + elt_idx * REG_EM_BLE_WL_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_WLBDADDR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_WLBDADDR_LSB    0
#define EM_BLE_WLBDADDR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_WLBDADDR_RST    0x0

__INLINE uint16_t em_ble_wl_bdaddr_wlbdaddr_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(EM_BLE_WL_BDADDR_ADDR + elt_idx * REG_EM_BLE_WL_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_wl_bdaddr_wlbdaddr_setf(int elt_idx, int reg_idx, uint16_t wlbdaddr)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)wlbdaddr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_WL_BDADDR_ADDR + elt_idx * REG_EM_BLE_WL_SIZE + reg_idx * 2, (uint16_t)wlbdaddr << 0);
}


#endif // _REG_EM_BLE_WL_H_

