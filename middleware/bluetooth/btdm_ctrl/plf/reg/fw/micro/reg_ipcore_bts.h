#ifndef _REG_IPCORE_BTS_H_
#define _REG_IPCORE_BTS_H_

#include <stdint.h>
#include "_reg_ipcore_bts.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_IPCORE_BTS_COUNT 125

#define REG_IPCORE_BTS_DECODING_MASK 0x000001FF

/**
 * @brief ISOCNTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31              ISOSAMP   0
 *     30              ISO_UPD   0
 *     02    ISO_CLKSHIFT_MODE   0
 *     01   ISO_PHASE_SHIFT_MODE   0
 *     00          ISOCORRMODE   0
 * </pre>
 */
#define IP_ISOCNTCNTL_ADDR   0x400909C0
#define IP_ISOCNTCNTL_OFFSET 0x000001C0
#define IP_ISOCNTCNTL_INDEX  0x00000070
#define IP_ISOCNTCNTL_RESET  0x00000000

__INLINE uint32_t ip_isocntcntl_get(void)
{
    return REG_IP_RD(IP_ISOCNTCNTL_ADDR);
}

__INLINE void ip_isocntcntl_set(uint32_t value)
{
    REG_IP_WR(IP_ISOCNTCNTL_ADDR, value);
}

// field definitions
#define IP_ISOSAMP_BIT                 ((uint32_t)0x80000000)
#define IP_ISOSAMP_POS                 31
#define IP_ISO_UPD_BIT                 ((uint32_t)0x40000000)
#define IP_ISO_UPD_POS                 30
#define IP_ISO_CLKSHIFT_MODE_BIT       ((uint32_t)0x00000004)
#define IP_ISO_CLKSHIFT_MODE_POS       2
#define IP_ISO_PHASE_SHIFT_MODE_BIT    ((uint32_t)0x00000002)
#define IP_ISO_PHASE_SHIFT_MODE_POS    1
#define IP_ISOCORRMODE_BIT             ((uint32_t)0x00000001)
#define IP_ISOCORRMODE_POS             0

#define IP_ISOSAMP_RST                 0x0
#define IP_ISO_UPD_RST                 0x0
#define IP_ISO_CLKSHIFT_MODE_RST       0x0
#define IP_ISO_PHASE_SHIFT_MODE_RST    0x0
#define IP_ISOCORRMODE_RST             0x0

__INLINE void ip_isocntcntl_pack(uint8_t isosamp, uint8_t isoupd, uint8_t isoclkshiftmode, uint8_t isophaseshiftmode, uint8_t isocorrmode)
{
    ASSERT_ERR((((uint32_t)isosamp << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)isoupd << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)isoclkshiftmode << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)isophaseshiftmode << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)isocorrmode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_ISOCNTCNTL_ADDR, ((uint32_t)isosamp << 31) | ((uint32_t)isoupd << 30) | ((uint32_t)isoclkshiftmode << 2) | ((uint32_t)isophaseshiftmode << 1) | ((uint32_t)isocorrmode << 0));
}

__INLINE void ip_isocntcntl_unpack(uint8_t *isosamp, uint8_t *isoupd, uint8_t *isoclkshiftmode, uint8_t *isophaseshiftmode, uint8_t *isocorrmode)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCNTL_ADDR);

    *isosamp = (localVal & ((uint32_t)0x80000000)) >> 31;
    *isoupd = (localVal & ((uint32_t)0x40000000)) >> 30;
    *isoclkshiftmode = (localVal & ((uint32_t)0x00000004)) >> 2;
    *isophaseshiftmode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *isocorrmode = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_isocntcntl_isosamp_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_isocntcntl_isosamp_setf(uint8_t isosamp)
{
    ASSERT_ERR((((uint32_t)isosamp << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_ISOCNTCNTL_ADDR, (REG_IP_RD(IP_ISOCNTCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)isosamp << 31));
}

__INLINE uint8_t ip_isocntcntl_iso_upd_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ip_isocntcntl_iso_upd_setf(uint8_t isoupd)
{
    ASSERT_ERR((((uint32_t)isoupd << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_IP_WR(IP_ISOCNTCNTL_ADDR, (REG_IP_RD(IP_ISOCNTCNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)isoupd << 30));
}

__INLINE uint8_t ip_isocntcntl_iso_clkshift_mode_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ip_isocntcntl_iso_clkshift_mode_setf(uint8_t isoclkshiftmode)
{
    ASSERT_ERR((((uint32_t)isoclkshiftmode << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_IP_WR(IP_ISOCNTCNTL_ADDR, (REG_IP_RD(IP_ISOCNTCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)isoclkshiftmode << 2));
}

__INLINE uint8_t ip_isocntcntl_iso_phase_shift_mode_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ip_isocntcntl_iso_phase_shift_mode_setf(uint8_t isophaseshiftmode)
{
    ASSERT_ERR((((uint32_t)isophaseshiftmode << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_IP_WR(IP_ISOCNTCNTL_ADDR, (REG_IP_RD(IP_ISOCNTCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)isophaseshiftmode << 1));
}

__INLINE uint8_t ip_isocntcntl_isocorrmode_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_isocntcntl_isocorrmode_setf(uint8_t isocorrmode)
{
    ASSERT_ERR((((uint32_t)isocorrmode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_ISOCNTCNTL_ADDR, (REG_IP_RD(IP_ISOCNTCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)isocorrmode << 0));
}

/**
 * @brief ISOCNTSAMP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           ISOCNTSAMP   0x0
 * </pre>
 */
#define IP_ISOCNTSAMP_ADDR   0x400909C4
#define IP_ISOCNTSAMP_OFFSET 0x000001C4
#define IP_ISOCNTSAMP_INDEX  0x00000071
#define IP_ISOCNTSAMP_RESET  0x00000000

__INLINE uint32_t ip_isocntsamp_get(void)
{
    return REG_IP_RD(IP_ISOCNTSAMP_ADDR);
}

__INLINE void ip_isocntsamp_set(uint32_t value)
{
    REG_IP_WR(IP_ISOCNTSAMP_ADDR, value);
}

// field definitions
#define IP_ISOCNTSAMP_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_ISOCNTSAMP_LSB    0
#define IP_ISOCNTSAMP_WIDTH  ((uint32_t)0x00000020)

#define IP_ISOCNTSAMP_RST    0x0

__INLINE uint32_t ip_isocntsamp_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTSAMP_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_isocntsamp_setf(uint32_t isocntsamp)
{
    ASSERT_ERR((((uint32_t)isocntsamp << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_ISOCNTSAMP_ADDR, (uint32_t)isocntsamp << 0);
}

/**
 * @brief ISOCNTCORR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           ISOCNTCORR   0x0
 * </pre>
 */
#define IP_ISOCNTCORR_ADDR   0x400909C8
#define IP_ISOCNTCORR_OFFSET 0x000001C8
#define IP_ISOCNTCORR_INDEX  0x00000072
#define IP_ISOCNTCORR_RESET  0x00000000

__INLINE uint32_t ip_isocntcorr_get(void)
{
    return REG_IP_RD(IP_ISOCNTCORR_ADDR);
}

__INLINE void ip_isocntcorr_set(uint32_t value)
{
    REG_IP_WR(IP_ISOCNTCORR_ADDR, value);
}

// field definitions
#define IP_ISOCNTCORR_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_ISOCNTCORR_LSB    0
#define IP_ISOCNTCORR_WIDTH  ((uint32_t)0x00000020)

#define IP_ISOCNTCORR_RST    0x0

__INLINE uint32_t ip_isocntcorr_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCORR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_isocntcorr_setf(uint32_t isocntcorr)
{
    ASSERT_ERR((((uint32_t)isocntcorr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_ISOCNTCORR_ADDR, (uint32_t)isocntcorr << 0);
}

/**
 * @brief ISOCNTCORR_HUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00       ISOCNTCORR_HUS   0
 * </pre>
 */
#define IP_ISOCNTCORR_HUS_ADDR   0x400909CC
#define IP_ISOCNTCORR_HUS_OFFSET 0x000001CC
#define IP_ISOCNTCORR_HUS_INDEX  0x00000073
#define IP_ISOCNTCORR_HUS_RESET  0x00000000

__INLINE uint32_t ip_isocntcorr_hus_get(void)
{
    return REG_IP_RD(IP_ISOCNTCORR_HUS_ADDR);
}

__INLINE void ip_isocntcorr_hus_set(uint32_t value)
{
    REG_IP_WR(IP_ISOCNTCORR_HUS_ADDR, value);
}

// field definitions
#define IP_ISOCNTCORR_HUS_BIT    ((uint32_t)0x00000001)
#define IP_ISOCNTCORR_HUS_POS    0

#define IP_ISOCNTCORR_HUS_RST    0x0

__INLINE uint8_t ip_isocntcorr_hus_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOCNTCORR_HUS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_isocntcorr_hus_setf(uint8_t isocntcorrhus)
{
    ASSERT_ERR((((uint32_t)isocntcorrhus << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_ISOCNTCORR_HUS_ADDR, (uint32_t)isocntcorrhus << 0);
}

/**
 * @brief ISOINTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00            ISOINTMSK   0x0
 * </pre>
 */
#define IP_ISOINTCNTL_ADDR   0x400909D0
#define IP_ISOINTCNTL_OFFSET 0x000001D0
#define IP_ISOINTCNTL_INDEX  0x00000074
#define IP_ISOINTCNTL_RESET  0x00000000

__INLINE uint32_t ip_isointcntl_get(void)
{
    return REG_IP_RD(IP_ISOINTCNTL_ADDR);
}

__INLINE void ip_isointcntl_set(uint32_t value)
{
    REG_IP_WR(IP_ISOINTCNTL_ADDR, value);
}

// field definitions
#define IP_ISOINTMSK_MASK   ((uint32_t)0x000000FF)
#define IP_ISOINTMSK_LSB    0
#define IP_ISOINTMSK_WIDTH  ((uint32_t)0x00000008)

#define IP_ISOINTMSK_RST    0x0

__INLINE uint8_t ip_isointcntl_isointmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOINTCNTL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_isointcntl_isointmsk_setf(uint8_t isointmsk)
{
    ASSERT_ERR((((uint32_t)isointmsk << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(IP_ISOINTCNTL_ADDR, (uint32_t)isointmsk << 0);
}

/**
 * @brief ISOINTSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00           ISOINTSTAT   0x0
 * </pre>
 */
#define IP_ISOINTSTAT_ADDR   0x400909D4
#define IP_ISOINTSTAT_OFFSET 0x000001D4
#define IP_ISOINTSTAT_INDEX  0x00000075
#define IP_ISOINTSTAT_RESET  0x00000000

__INLINE uint32_t ip_isointstat_get(void)
{
    return REG_IP_RD(IP_ISOINTSTAT_ADDR);
}

// field definitions
#define IP_ISOINTSTAT_MASK   ((uint32_t)0x000000FF)
#define IP_ISOINTSTAT_LSB    0
#define IP_ISOINTSTAT_WIDTH  ((uint32_t)0x00000008)

#define IP_ISOINTSTAT_RST    0x0

__INLINE uint8_t ip_isointstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOINTSTAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ISOINTACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00            ISOINTACK   0x0
 * </pre>
 */
#define IP_ISOINTACK_ADDR   0x400909D8
#define IP_ISOINTACK_OFFSET 0x000001D8
#define IP_ISOINTACK_INDEX  0x00000076
#define IP_ISOINTACK_RESET  0x00000000

__INLINE uint32_t ip_isointack_get(void)
{
    return REG_IP_RD(IP_ISOINTACK_ADDR);
}

__INLINE void ip_isointack_clear(uint32_t value)
{
    REG_IP_WR(IP_ISOINTACK_ADDR, value);
}

// field definitions
#define IP_ISOINTACK_MASK   ((uint32_t)0x000000FF)
#define IP_ISOINTACK_LSB    0
#define IP_ISOINTACK_WIDTH  ((uint32_t)0x00000008)

#define IP_ISOINTACK_RST    0x0

__INLINE uint8_t ip_isointack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOINTACK_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_isointack_clearf(uint8_t isointack)
{
    ASSERT_ERR((((uint32_t)isointack << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(IP_ISOINTACK_ADDR, (uint32_t)isointack << 0);
}

/**
 * @brief ISOGPIOCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           ISOGPIOBEH   0
 *  07:00           ISOGPIOMSK   0x0
 * </pre>
 */
#define IP_ISOGPIOCNTL_ADDR   0x400909E0
#define IP_ISOGPIOCNTL_OFFSET 0x000001E0
#define IP_ISOGPIOCNTL_INDEX  0x00000078
#define IP_ISOGPIOCNTL_RESET  0x00000000

__INLINE uint32_t ip_isogpiocntl_get(void)
{
    return REG_IP_RD(IP_ISOGPIOCNTL_ADDR);
}

__INLINE void ip_isogpiocntl_set(uint32_t value)
{
    REG_IP_WR(IP_ISOGPIOCNTL_ADDR, value);
}

// field definitions
#define IP_ISOGPIOBEH_BIT    ((uint32_t)0x80000000)
#define IP_ISOGPIOBEH_POS    31
#define IP_ISOGPIOMSK_MASK   ((uint32_t)0x000000FF)
#define IP_ISOGPIOMSK_LSB    0
#define IP_ISOGPIOMSK_WIDTH  ((uint32_t)0x00000008)

#define IP_ISOGPIOBEH_RST    0x0
#define IP_ISOGPIOMSK_RST    0x0

__INLINE void ip_isogpiocntl_pack(uint8_t isogpiobeh, uint8_t isogpiomsk)
{
    ASSERT_ERR((((uint32_t)isogpiobeh << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)isogpiomsk << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(IP_ISOGPIOCNTL_ADDR, ((uint32_t)isogpiobeh << 31) | ((uint32_t)isogpiomsk << 0));
}

__INLINE void ip_isogpiocntl_unpack(uint8_t *isogpiobeh, uint8_t *isogpiomsk)
{
    uint32_t localVal = REG_IP_RD(IP_ISOGPIOCNTL_ADDR);

    *isogpiobeh = (localVal & ((uint32_t)0x80000000)) >> 31;
    *isogpiomsk = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ip_isogpiocntl_isogpiobeh_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOGPIOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_isogpiocntl_isogpiobeh_setf(uint8_t isogpiobeh)
{
    ASSERT_ERR((((uint32_t)isogpiobeh << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_ISOGPIOCNTL_ADDR, (REG_IP_RD(IP_ISOGPIOCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)isogpiobeh << 31));
}

__INLINE uint8_t ip_isogpiocntl_isogpiomsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ISOGPIOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ip_isogpiocntl_isogpiomsk_setf(uint8_t isogpiomsk)
{
    ASSERT_ERR((((uint32_t)isogpiomsk << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(IP_ISOGPIOCNTL_ADDR, (REG_IP_RD(IP_ISOGPIOCNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)isogpiomsk << 0));
}

/**
 * @brief ISOTIMERTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          ISOTIMERTGT   0x0
 * </pre>
 */
#define IP_ISOTIMERTGT_ADDR   0x400909F0
#define IP_ISOTIMERTGT_OFFSET 0x000001F0
#define IP_ISOTIMERTGT_INDEX  0x0000007C
#define IP_ISOTIMERTGT_RESET  0x00000000
#define IP_ISOTIMERTGT_COUNT  8

__INLINE uint32_t ip_isotimertgt_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    return REG_IP_RD(IP_ISOTIMERTGT_ADDR + reg_idx * 4);
}

__INLINE void ip_isotimertgt_set(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 7);
    REG_IP_WR(IP_ISOTIMERTGT_ADDR + reg_idx * 4, value);
}

// field definitions
#define IP_ISOTIMERTGT_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_ISOTIMERTGT_LSB    0
#define IP_ISOTIMERTGT_WIDTH  ((uint32_t)0x00000020)

#define IP_ISOTIMERTGT_RST    0x0

__INLINE uint32_t ip_isotimertgt_getf(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    uint32_t localVal = REG_IP_RD(IP_ISOTIMERTGT_ADDR + reg_idx * 4);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_isotimertgt_setf(int reg_idx, uint32_t isotimertgt)
{
    ASSERT_ERR(reg_idx <= 7);
    ASSERT_ERR((((uint32_t)isotimertgt << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_ISOTIMERTGT_ADDR + reg_idx * 4, (uint32_t)isotimertgt << 0);
}


#endif // _REG_IPCORE_BTS_H_

