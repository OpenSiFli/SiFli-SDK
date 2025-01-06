#ifndef _REG_EM_ET_H_
#define _REG_EM_ET_H_

#include <stdint.h>
#include "_reg_em_et.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_ET_COUNT 8

#define REG_EM_ET_DECODING_MASK 0x0000000F

#define REG_EM_ET_ADDR_GET(idx) (EM_ET_OFFSET + (idx) * REG_EM_ET_SIZE)

/**
 * @brief BT_EXTAB register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:11            SCH_PRIO1   0x0
 *     10                  SPA   0
 *     09                  CSB   0
 *     08                SNIFF   0
 *     07                 RSVD   0
 *     06                 eSCO   0
 *  05:03               STATUS   0x0
 *  02:00                 MODE   0x0
 * </pre>
 */
#define EM_BT_EXTAB_ADDR   (0x20408000 + EM_ET_OFFSET)
#define EM_BT_EXTAB_INDEX  0x00000000
#define EM_BT_EXTAB_RESET  0x00000000

__INLINE uint16_t em_bt_extab_get(int elt_idx)
{
    return EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_bt_extab_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_SCH_PRIO1_MASK   ((uint16_t)0x0000F800)
#define EM_SCH_PRIO1_LSB    11
#define EM_SCH_PRIO1_WIDTH  ((uint16_t)0x00000005)
#define EM_SPA_BIT          ((uint16_t)0x00000400)
#define EM_SPA_POS          10
#define EM_CSB_BIT          ((uint16_t)0x00000200)
#define EM_CSB_POS          9
#define EM_SNIFF_BIT        ((uint16_t)0x00000100)
#define EM_SNIFF_POS        8
#define EM_RSVD_BIT         ((uint16_t)0x00000080)
#define EM_RSVD_POS         7
#define EM_E_SCO_BIT        ((uint16_t)0x00000040)
#define EM_E_SCO_POS        6
#define EM_STATUS_MASK      ((uint16_t)0x00000038)
#define EM_STATUS_LSB       3
#define EM_STATUS_WIDTH     ((uint16_t)0x00000003)
#define EM_MODE_MASK        ((uint16_t)0x00000007)
#define EM_MODE_LSB         0
#define EM_MODE_WIDTH       ((uint16_t)0x00000003)

#define EM_SCH_PRIO1_RST    0x0
#define EM_SPA_RST          0x0
#define EM_CSB_RST          0x0
#define EM_SNIFF_RST        0x0
#define EM_RSVD_RST         0x0
#define EM_E_SCO_RST        0x0
#define EM_STATUS_RST       0x0
#define EM_MODE_RST         0x0

__INLINE void em_bt_extab_pack(int elt_idx, uint8_t schprio1, uint8_t spa, uint8_t csb, uint8_t sniff, uint8_t rsvd, uint8_t esco, uint8_t status, uint8_t mode)
{
    ASSERT_ERR((((uint16_t)schprio1 << 11) & ~((uint16_t)0x0000F800)) == 0);
    ASSERT_ERR((((uint16_t)spa << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)csb << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)sniff << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)rsvd << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)esco << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)status << 3) & ~((uint16_t)0x00000038)) == 0);
    ASSERT_ERR((((uint16_t)mode << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, ((uint16_t)schprio1 << 11) | ((uint16_t)spa << 10) | ((uint16_t)csb << 9) | ((uint16_t)sniff << 8) | ((uint16_t)rsvd << 7) | ((uint16_t)esco << 6) | ((uint16_t)status << 3) | ((uint16_t)mode << 0));
}

__INLINE void em_bt_extab_unpack(int elt_idx, uint8_t *schprio1, uint8_t *spa, uint8_t *csb, uint8_t *sniff, uint8_t *rsvd, uint8_t *esco, uint8_t *status, uint8_t *mode)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);

    *schprio1 = (localVal & ((uint16_t)0x0000F800)) >> 11;
    *spa = (localVal & ((uint16_t)0x00000400)) >> 10;
    *csb = (localVal & ((uint16_t)0x00000200)) >> 9;
    *sniff = (localVal & ((uint16_t)0x00000100)) >> 8;
    *rsvd = (localVal & ((uint16_t)0x00000080)) >> 7;
    *esco = (localVal & ((uint16_t)0x00000040)) >> 6;
    *status = (localVal & ((uint16_t)0x00000038)) >> 3;
    *mode = (localVal & ((uint16_t)0x00000007)) >> 0;
}

__INLINE uint8_t em_bt_extab_sch_prio1_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x0000F800)) >> 11);
}

__INLINE void em_bt_extab_sch_prio1_setf(int elt_idx, uint8_t schprio1)
{
    ASSERT_ERR((((uint16_t)schprio1 << 11) & ~((uint16_t)0x0000F800)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x0000F800)) | ((uint16_t)schprio1 << 11));
}

__INLINE uint8_t em_bt_extab_spa_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_bt_extab_spa_setf(int elt_idx, uint8_t spa)
{
    ASSERT_ERR((((uint16_t)spa << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)spa << 10));
}

__INLINE uint8_t em_bt_extab_csb_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_bt_extab_csb_setf(int elt_idx, uint8_t csb)
{
    ASSERT_ERR((((uint16_t)csb << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)csb << 9));
}

__INLINE uint8_t em_bt_extab_sniff_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_bt_extab_sniff_setf(int elt_idx, uint8_t sniff)
{
    ASSERT_ERR((((uint16_t)sniff << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)sniff << 8));
}

__INLINE uint8_t em_bt_extab_rsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_bt_extab_rsvd_setf(int elt_idx, uint8_t rsvd)
{
    ASSERT_ERR((((uint16_t)rsvd << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)rsvd << 7));
}

__INLINE uint8_t em_bt_extab_e_sco_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_bt_extab_e_sco_setf(int elt_idx, uint8_t esco)
{
    ASSERT_ERR((((uint16_t)esco << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)esco << 6));
}

__INLINE uint8_t em_bt_extab_status_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000038)) >> 3);
}

__INLINE void em_bt_extab_status_setf(int elt_idx, uint8_t status)
{
    ASSERT_ERR((((uint16_t)status << 3) & ~((uint16_t)0x00000038)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000038)) | ((uint16_t)status << 3));
}

__INLINE uint8_t em_bt_extab_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000007)) >> 0);
}

__INLINE void em_bt_extab_mode_setf(int elt_idx, uint8_t mode)
{
    ASSERT_ERR((((uint16_t)mode << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_WR(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_BT_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000007)) | ((uint16_t)mode << 0));
}

/**
 * @brief LE_EXTAB register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:11            SCH_PRIO1   0x0
 *     10                  SPA   0
 *     09                  SIC   0
 *     08               AE_NPS   0
 *     07                 RSVD   0
 *     06                  ISO   0
 *  05:03               STATUS   0x0
 *  02:00                 MODE   0x0
 * </pre>
 */
#define EM_LE_EXTAB_ADDR   (0x20408000 + EM_ET_OFFSET)
#define EM_LE_EXTAB_INDEX  0x00000000
#define EM_LE_EXTAB_RESET  0x00000000

__INLINE uint16_t em_le_extab_get(int elt_idx)
{
    return EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_le_extab_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_SCH_PRIO1_MASK   ((uint16_t)0x0000F800)
#define EM_SCH_PRIO1_LSB    11
#define EM_SCH_PRIO1_WIDTH  ((uint16_t)0x00000005)
#define EM_SPA_BIT          ((uint16_t)0x00000400)
#define EM_SPA_POS          10
#define EM_SIC_BIT          ((uint16_t)0x00000200)
#define EM_SIC_POS          9
#define EM_AE_NPS_BIT       ((uint16_t)0x00000100)
#define EM_AE_NPS_POS       8
#define EM_RSVD_BIT         ((uint16_t)0x00000080)
#define EM_RSVD_POS         7
#define EM_ISO_BIT          ((uint16_t)0x00000040)
#define EM_ISO_POS          6
#define EM_STATUS_MASK      ((uint16_t)0x00000038)
#define EM_STATUS_LSB       3
#define EM_STATUS_WIDTH     ((uint16_t)0x00000003)
#define EM_MODE_MASK        ((uint16_t)0x00000007)
#define EM_MODE_LSB         0
#define EM_MODE_WIDTH       ((uint16_t)0x00000003)

#define EM_SCH_PRIO1_RST    0x0
#define EM_SPA_RST          0x0
#define EM_SIC_RST          0x0
#define EM_AE_NPS_RST       0x0
#define EM_RSVD_RST         0x0
#define EM_ISO_RST          0x0
#define EM_STATUS_RST       0x0
#define EM_MODE_RST         0x0

__INLINE void em_le_extab_pack(int elt_idx, uint8_t schprio1, uint8_t spa, uint8_t sic, uint8_t aenps, uint8_t rsvd, uint8_t iso, uint8_t status, uint8_t mode)
{
    ASSERT_ERR((((uint16_t)schprio1 << 11) & ~((uint16_t)0x0000F800)) == 0);
    ASSERT_ERR((((uint16_t)spa << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)sic << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)aenps << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)rsvd << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)iso << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)status << 3) & ~((uint16_t)0x00000038)) == 0);
    ASSERT_ERR((((uint16_t)mode << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, ((uint16_t)schprio1 << 11) | ((uint16_t)spa << 10) | ((uint16_t)sic << 9) | ((uint16_t)aenps << 8) | ((uint16_t)rsvd << 7) | ((uint16_t)iso << 6) | ((uint16_t)status << 3) | ((uint16_t)mode << 0));
}

__INLINE void em_le_extab_unpack(int elt_idx, uint8_t *schprio1, uint8_t *spa, uint8_t *sic, uint8_t *aenps, uint8_t *rsvd, uint8_t *iso, uint8_t *status, uint8_t *mode)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);

    *schprio1 = (localVal & ((uint16_t)0x0000F800)) >> 11;
    *spa = (localVal & ((uint16_t)0x00000400)) >> 10;
    *sic = (localVal & ((uint16_t)0x00000200)) >> 9;
    *aenps = (localVal & ((uint16_t)0x00000100)) >> 8;
    *rsvd = (localVal & ((uint16_t)0x00000080)) >> 7;
    *iso = (localVal & ((uint16_t)0x00000040)) >> 6;
    *status = (localVal & ((uint16_t)0x00000038)) >> 3;
    *mode = (localVal & ((uint16_t)0x00000007)) >> 0;
}

__INLINE uint8_t em_le_extab_sch_prio1_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x0000F800)) >> 11);
}

__INLINE void em_le_extab_sch_prio1_setf(int elt_idx, uint8_t schprio1)
{
    ASSERT_ERR((((uint16_t)schprio1 << 11) & ~((uint16_t)0x0000F800)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x0000F800)) | ((uint16_t)schprio1 << 11));
}

__INLINE uint8_t em_le_extab_spa_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_le_extab_spa_setf(int elt_idx, uint8_t spa)
{
    ASSERT_ERR((((uint16_t)spa << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)spa << 10));
}

__INLINE uint8_t em_le_extab_sic_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_le_extab_sic_setf(int elt_idx, uint8_t sic)
{
    ASSERT_ERR((((uint16_t)sic << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)sic << 9));
}

__INLINE uint8_t em_le_extab_ae_nps_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_le_extab_ae_nps_setf(int elt_idx, uint8_t aenps)
{
    ASSERT_ERR((((uint16_t)aenps << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)aenps << 8));
}

__INLINE uint8_t em_le_extab_rsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_le_extab_rsvd_setf(int elt_idx, uint8_t rsvd)
{
    ASSERT_ERR((((uint16_t)rsvd << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)rsvd << 7));
}

__INLINE uint8_t em_le_extab_iso_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_le_extab_iso_setf(int elt_idx, uint8_t iso)
{
    ASSERT_ERR((((uint16_t)iso << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)iso << 6));
}

__INLINE uint8_t em_le_extab_status_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000038)) >> 3);
}

__INLINE void em_le_extab_status_setf(int elt_idx, uint8_t status)
{
    ASSERT_ERR((((uint16_t)status << 3) & ~((uint16_t)0x00000038)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000038)) | ((uint16_t)status << 3));
}

__INLINE uint8_t em_le_extab_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000007)) >> 0);
}

__INLINE void em_le_extab_mode_setf(int elt_idx, uint8_t mode)
{
    ASSERT_ERR((((uint16_t)mode << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_WR(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_LE_EXTAB_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000007)) | ((uint16_t)mode << 0));
}

/**
 * @brief RAWSTP0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              RAWSTP0   0x0
 * </pre>
 */
#define EM_RAWSTP0_ADDR   (0x20408002 + EM_ET_OFFSET)
#define EM_RAWSTP0_INDEX  0x00000001
#define EM_RAWSTP0_RESET  0x00000000

__INLINE uint16_t em_rawstp0_get(int elt_idx)
{
    return EM_RD(EM_RAWSTP0_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_rawstp0_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_RAWSTP0_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_RAWSTP0_MASK   ((uint16_t)0x0000FFFF)
#define EM_RAWSTP0_LSB    0
#define EM_RAWSTP0_WIDTH  ((uint16_t)0x00000010)

#define EM_RAWSTP0_RST    0x0

__INLINE uint16_t em_rawstp0_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_RAWSTP0_ADDR + elt_idx * REG_EM_ET_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_rawstp0_setf(int elt_idx, uint16_t rawstp0)
{
    ASSERT_ERR((((uint16_t)rawstp0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_WR(EM_RAWSTP0_ADDR + elt_idx * REG_EM_ET_SIZE, (uint16_t)rawstp0 << 0);
}

/**
 * @brief RAWSTP1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:00              RAWSTP1   0x0
 * </pre>
 */
#define EM_RAWSTP1_ADDR   (0x20408004 + EM_ET_OFFSET)
#define EM_RAWSTP1_INDEX  0x00000002
#define EM_RAWSTP1_RESET  0x00000000

__INLINE uint16_t em_rawstp1_get(int elt_idx)
{
    return EM_RD(EM_RAWSTP1_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_rawstp1_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_RAWSTP1_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_RAWSTP1_MASK   ((uint16_t)0x00000FFF)
#define EM_RAWSTP1_LSB    0
#define EM_RAWSTP1_WIDTH  ((uint16_t)0x0000000C)

#define EM_RAWSTP1_RST    0x0

__INLINE uint16_t em_rawstp1_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_RAWSTP1_ADDR + elt_idx * REG_EM_ET_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00000FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_rawstp1_setf(int elt_idx, uint16_t rawstp1)
{
    ASSERT_ERR((((uint16_t)rawstp1 << 0) & ~((uint16_t)0x00000FFF)) == 0);
    EM_WR(EM_RAWSTP1_ADDR + elt_idx * REG_EM_ET_SIZE, (uint16_t)rawstp1 << 0);
}

/**
 * @brief FINESTP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00              FINESTP   0x0
 * </pre>
 */
#define EM_FINESTP_ADDR   (0x20408006 + EM_ET_OFFSET)
#define EM_FINESTP_INDEX  0x00000003
#define EM_FINESTP_RESET  0x00000000

__INLINE uint16_t em_finestp_get(int elt_idx)
{
    return EM_RD(EM_FINESTP_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_finestp_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_FINESTP_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_FINESTP_MASK   ((uint16_t)0x000003FF)
#define EM_FINESTP_LSB    0
#define EM_FINESTP_WIDTH  ((uint16_t)0x0000000A)

#define EM_FINESTP_RST    0x0

__INLINE uint16_t em_finestp_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_FINESTP_ADDR + elt_idx * REG_EM_ET_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_finestp_setf(int elt_idx, uint16_t finestp)
{
    ASSERT_ERR((((uint16_t)finestp << 0) & ~((uint16_t)0x000003FF)) == 0);
    EM_WR(EM_FINESTP_ADDR + elt_idx * REG_EM_ET_SIZE, (uint16_t)finestp << 0);
}

/**
 * @brief CSPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00                CSPTR   0x0
 * </pre>
 */
#define EM_CSPTR_ADDR   (0x20408008 + EM_ET_OFFSET)
#define EM_CSPTR_INDEX  0x00000004
#define EM_CSPTR_RESET  0x00000000

__INLINE uint16_t em_csptr_get(int elt_idx)
{
    return EM_RD(EM_CSPTR_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_csptr_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_CSPTR_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_CSPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_CSPTR_LSB    0
#define EM_CSPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_CSPTR_RST    0x0

__INLINE uint16_t em_csptr_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_CSPTR_ADDR + elt_idx * REG_EM_ET_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_csptr_setf(int elt_idx, uint16_t csptr)
{
    ASSERT_ERR((((uint16_t)csptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_WR(EM_CSPTR_ADDR + elt_idx * REG_EM_ET_SIZE, (uint16_t)csptr << 0);
}

/**
 * @brief PRIOBW register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15          PRIO1D_UNIT   0
 *  14:00               PRIO1D   0x0
 * </pre>
 */
#define EM_PRIOBW_ADDR   (0x2040800A + EM_ET_OFFSET)
#define EM_PRIOBW_INDEX  0x00000005
#define EM_PRIOBW_RESET  0x00000000

__INLINE uint16_t em_priobw_get(int elt_idx)
{
    return EM_RD(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_priobw_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_PRIO1D_UNIT_BIT    ((uint16_t)0x00008000)
#define EM_PRIO1D_UNIT_POS    15
#define EM_PRIO1D_MASK        ((uint16_t)0x00007FFF)
#define EM_PRIO1D_LSB         0
#define EM_PRIO1D_WIDTH       ((uint16_t)0x0000000F)

#define EM_PRIO1D_UNIT_RST    0x0
#define EM_PRIO1D_RST         0x0

__INLINE void em_priobw_pack(int elt_idx, uint8_t prio1dunit, uint16_t prio1d)
{
    ASSERT_ERR((((uint16_t)prio1dunit << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)prio1d << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_WR(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE, ((uint16_t)prio1dunit << 15) | ((uint16_t)prio1d << 0));
}

__INLINE void em_priobw_unpack(int elt_idx, uint8_t *prio1dunit, uint16_t *prio1d)
{
    uint16_t localVal = EM_RD(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE);

    *prio1dunit = (localVal & ((uint16_t)0x00008000)) >> 15;
    *prio1d = (localVal & ((uint16_t)0x00007FFF)) >> 0;
}

__INLINE uint8_t em_priobw_prio1d_unit_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_priobw_prio1d_unit_setf(int elt_idx, uint8_t prio1dunit)
{
    ASSERT_ERR((((uint16_t)prio1dunit << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_WR(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)prio1dunit << 15));
}

__INLINE uint16_t em_priobw_prio1d_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00007FFF)) >> 0);
}

__INLINE void em_priobw_prio1d_setf(int elt_idx, uint16_t prio1d)
{
    ASSERT_ERR((((uint16_t)prio1d << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_WR(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_PRIOBW_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00007FFF)) | ((uint16_t)prio1d << 0));
}

/**
 * @brief PRIOLVL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  12:08            SCH_PRIO3   0x0
 *  04:00            SCH_PRIO2   0x0
 * </pre>
 */
#define EM_PRIOLVL_ADDR   (0x2040800C + EM_ET_OFFSET)
#define EM_PRIOLVL_INDEX  0x00000006
#define EM_PRIOLVL_RESET  0x00000000

__INLINE uint16_t em_priolvl_get(int elt_idx)
{
    return EM_RD(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_priolvl_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_SCH_PRIO3_MASK   ((uint16_t)0x00001F00)
#define EM_SCH_PRIO3_LSB    8
#define EM_SCH_PRIO3_WIDTH  ((uint16_t)0x00000005)
#define EM_SCH_PRIO2_MASK   ((uint16_t)0x0000001F)
#define EM_SCH_PRIO2_LSB    0
#define EM_SCH_PRIO2_WIDTH  ((uint16_t)0x00000005)

#define EM_SCH_PRIO3_RST    0x0
#define EM_SCH_PRIO2_RST    0x0

__INLINE void em_priolvl_pack(int elt_idx, uint8_t schprio3, uint8_t schprio2)
{
    ASSERT_ERR((((uint16_t)schprio3 << 8) & ~((uint16_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint16_t)schprio2 << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_WR(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE, ((uint16_t)schprio3 << 8) | ((uint16_t)schprio2 << 0));
}

__INLINE void em_priolvl_unpack(int elt_idx, uint8_t *schprio3, uint8_t *schprio2)
{
    uint16_t localVal = EM_RD(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE);

    *schprio3 = (localVal & ((uint16_t)0x00001F00)) >> 8;
    *schprio2 = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_priolvl_sch_prio3_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00001F00)) >> 8);
}

__INLINE void em_priolvl_sch_prio3_setf(int elt_idx, uint8_t schprio3)
{
    ASSERT_ERR((((uint16_t)schprio3 << 8) & ~((uint16_t)0x00001F00)) == 0);
    EM_WR(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00001F00)) | ((uint16_t)schprio3 << 8));
}

__INLINE uint8_t em_priolvl_sch_prio2_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_priolvl_sch_prio2_setf(int elt_idx, uint8_t schprio2)
{
    ASSERT_ERR((((uint16_t)schprio2 << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_WR(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_PRIOLVL_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)schprio2 << 0));
}

/**
 * @brief PTI_VXCHAN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  12:08             PTI_PRIO   0x0
 *  01:00               VXCHAN   0x0
 * </pre>
 */
#define EM_PTI_VXCHAN_ADDR   (0x2040800E + EM_ET_OFFSET)
#define EM_PTI_VXCHAN_INDEX  0x00000007
#define EM_PTI_VXCHAN_RESET  0x00000000

__INLINE uint16_t em_pti_vxchan_get(int elt_idx)
{
    return EM_RD(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE);
}

__INLINE void em_pti_vxchan_set(int elt_idx, uint16_t value)
{
    EM_WR(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE, value);
}

// field definitions
#define EM_PTI_PRIO_MASK   ((uint16_t)0x00001F00)
#define EM_PTI_PRIO_LSB    8
#define EM_PTI_PRIO_WIDTH  ((uint16_t)0x00000005)
#define EM_VXCHAN_MASK     ((uint16_t)0x00000003)
#define EM_VXCHAN_LSB      0
#define EM_VXCHAN_WIDTH    ((uint16_t)0x00000002)

#define EM_PTI_PRIO_RST    0x0
#define EM_VXCHAN_RST      0x0

__INLINE void em_pti_vxchan_pack(int elt_idx, uint8_t ptiprio, uint8_t vxchan)
{
    ASSERT_ERR((((uint16_t)ptiprio << 8) & ~((uint16_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint16_t)vxchan << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_WR(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE, ((uint16_t)ptiprio << 8) | ((uint16_t)vxchan << 0));
}

__INLINE void em_pti_vxchan_unpack(int elt_idx, uint8_t *ptiprio, uint8_t *vxchan)
{
    uint16_t localVal = EM_RD(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE);

    *ptiprio = (localVal & ((uint16_t)0x00001F00)) >> 8;
    *vxchan = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_pti_vxchan_pti_prio_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00001F00)) >> 8);
}

__INLINE void em_pti_vxchan_pti_prio_setf(int elt_idx, uint8_t ptiprio)
{
    ASSERT_ERR((((uint16_t)ptiprio << 8) & ~((uint16_t)0x00001F00)) == 0);
    EM_WR(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00001F00)) | ((uint16_t)ptiprio << 8));
}

__INLINE uint8_t em_pti_vxchan_vxchan_getf(int elt_idx)
{
    uint16_t localVal = EM_RD(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_pti_vxchan_vxchan_setf(int elt_idx, uint8_t vxchan)
{
    ASSERT_ERR((((uint16_t)vxchan << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_WR(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE, (EM_RD(EM_PTI_VXCHAN_ADDR + elt_idx * REG_EM_ET_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)vxchan << 0));
}


#endif // _REG_EM_ET_H_

