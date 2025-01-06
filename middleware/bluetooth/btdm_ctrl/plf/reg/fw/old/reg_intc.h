#ifndef _REG_INTC_H_
#define _REG_INTC_H_

#include <stdint.h>
#include "_reg_intc.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_INTC_COUNT 70

#define REG_INTC_DECODING_MASK 0x000001FF

/**
 * @brief IRQSTATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00              IRQSTAT   0x0
 * </pre>
 */
#define INTC_IRQSTATUS_ADDR   0x10001000
#define INTC_IRQSTATUS_OFFSET 0x00000000
#define INTC_IRQSTATUS_INDEX  0x00000000
#define INTC_IRQSTATUS_RESET  0x00000000

__INLINE uint32_t intc_irqstatus_get(void)
{
    return REG_PL_RD(INTC_IRQSTATUS_ADDR);
}

// field definitions
#define INTC_IRQSTAT_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_IRQSTAT_LSB    0
#define INTC_IRQSTAT_WIDTH  ((uint32_t)0x00000020)

#define INTC_IRQSTAT_RST    0x0

__INLINE uint32_t intc_irqstatus_irqstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_IRQSTATUS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief IRQRAWSTATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           IRQRAWSTAT   0x0
 * </pre>
 */
#define INTC_IRQRAWSTATUS_ADDR   0x10001004
#define INTC_IRQRAWSTATUS_OFFSET 0x00000004
#define INTC_IRQRAWSTATUS_INDEX  0x00000001
#define INTC_IRQRAWSTATUS_RESET  0x00000000

__INLINE uint32_t intc_irqrawstatus_get(void)
{
    return REG_PL_RD(INTC_IRQRAWSTATUS_ADDR);
}

// field definitions
#define INTC_IRQRAWSTAT_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_IRQRAWSTAT_LSB    0
#define INTC_IRQRAWSTAT_WIDTH  ((uint32_t)0x00000020)

#define INTC_IRQRAWSTAT_RST    0x0

__INLINE uint32_t intc_irqrawstatus_irqrawstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_IRQRAWSTATUS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief IRQENABLESET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         IRQENABLESET   0x0
 * </pre>
 */
#define INTC_IRQENABLESET_ADDR   0x10001008
#define INTC_IRQENABLESET_OFFSET 0x00000008
#define INTC_IRQENABLESET_INDEX  0x00000002
#define INTC_IRQENABLESET_RESET  0x00000000

__INLINE uint32_t intc_irqenableset_get(void)
{
    return REG_PL_RD(INTC_IRQENABLESET_ADDR);
}

__INLINE void intc_irqenableset_set(uint32_t value)
{
    REG_PL_WR(INTC_IRQENABLESET_ADDR, value);
}

// field definitions
#define INTC_IRQENABLESET_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_IRQENABLESET_LSB    0
#define INTC_IRQENABLESET_WIDTH  ((uint32_t)0x00000020)

#define INTC_IRQENABLESET_RST    0x0

__INLINE uint32_t intc_irqenableset_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_IRQENABLESET_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void intc_irqenableset_setf(uint32_t irqenableset)
{
    ASSERT_ERR((((uint32_t)irqenableset << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(INTC_IRQENABLESET_ADDR, (uint32_t)irqenableset << 0);
}

/**
 * @brief IRQENABLECLEAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00       IRQENABLECLEAR   0x0
 * </pre>
 */
#define INTC_IRQENABLECLEAR_ADDR   0x1000100C
#define INTC_IRQENABLECLEAR_OFFSET 0x0000000C
#define INTC_IRQENABLECLEAR_INDEX  0x00000003
#define INTC_IRQENABLECLEAR_RESET  0x00000000

__INLINE void intc_irqenableclear_set(uint32_t value)
{
    REG_PL_WR(INTC_IRQENABLECLEAR_ADDR, value);
}

// field definitions
#define INTC_IRQENABLECLEAR_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_IRQENABLECLEAR_LSB    0
#define INTC_IRQENABLECLEAR_WIDTH  ((uint32_t)0x00000020)

#define INTC_IRQENABLECLEAR_RST    0x0

__INLINE void intc_irqenableclear_setf(uint32_t irqenableclear)
{
    ASSERT_ERR((((uint32_t)irqenableclear << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(INTC_IRQENABLECLEAR_ADDR, (uint32_t)irqenableclear << 0);
}

/**
 * @brief IRQSOFT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02               IRQSFT   0
 * </pre>
 */
#define INTC_IRQSOFT_ADDR   0x10001010
#define INTC_IRQSOFT_OFFSET 0x00000010
#define INTC_IRQSOFT_INDEX  0x00000004
#define INTC_IRQSOFT_RESET  0x00000000

__INLINE uint32_t intc_irqsoft_get(void)
{
    return REG_PL_RD(INTC_IRQSOFT_ADDR);
}

__INLINE void intc_irqsoft_set(uint32_t value)
{
    REG_PL_WR(INTC_IRQSOFT_ADDR, value);
}

// field definitions
#define INTC_IRQSFT_BIT    ((uint32_t)0x00000004)
#define INTC_IRQSFT_POS    2

#define INTC_IRQSFT_RST    0x0

__INLINE void intc_irqsoft_irqsft_setf(uint8_t irqsft)
{
    ASSERT_ERR((((uint32_t)irqsft << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(INTC_IRQSOFT_ADDR, (uint32_t)irqsft << 2);
}

/**
 * @brief IRQACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               IRQACK   0x0
 * </pre>
 */
#define INTC_IRQACK_ADDR   0x10001014
#define INTC_IRQACK_OFFSET 0x00000014
#define INTC_IRQACK_INDEX  0x00000005
#define INTC_IRQACK_RESET  0x00000000

__INLINE void intc_irqack_set(uint32_t value)
{
    REG_PL_WR(INTC_IRQACK_ADDR, value);
}

// field definitions
#define INTC_IRQACK_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_IRQACK_LSB    0
#define INTC_IRQACK_WIDTH  ((uint32_t)0x00000020)

#define INTC_IRQACK_RST    0x0

__INLINE void intc_irqack_setf(uint32_t irqack)
{
    ASSERT_ERR((((uint32_t)irqack << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(INTC_IRQACK_ADDR, (uint32_t)irqack << 0);
}

/**
 * @brief IRQINDEX register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  04:00             IRQINDEX   0x0
 * </pre>
 */
#define INTC_IRQINDEX_ADDR   0x1000101C
#define INTC_IRQINDEX_OFFSET 0x0000001C
#define INTC_IRQINDEX_INDEX  0x00000007
#define INTC_IRQINDEX_RESET  0x00000000

__INLINE uint32_t intc_irqindex_get(void)
{
    return REG_PL_RD(INTC_IRQINDEX_ADDR);
}

// field definitions
#define INTC_IRQINDEX_MASK   ((uint32_t)0x0000001F)
#define INTC_IRQINDEX_LSB    0
#define INTC_IRQINDEX_WIDTH  ((uint32_t)0x00000005)

#define INTC_IRQINDEX_RST    0x0

__INLINE uint8_t intc_irqindex_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_IRQINDEX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000001F)) == 0);
    return (localVal >> 0);
}

/**
 * @brief FIQSTATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00              FIQSTAT   0x0
 * </pre>
 */
#define INTC_FIQSTATUS_ADDR   0x10001100
#define INTC_FIQSTATUS_OFFSET 0x00000100
#define INTC_FIQSTATUS_INDEX  0x00000040
#define INTC_FIQSTATUS_RESET  0x00000000

__INLINE uint32_t intc_fiqstatus_get(void)
{
    return REG_PL_RD(INTC_FIQSTATUS_ADDR);
}

// field definitions
#define INTC_FIQSTAT_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_FIQSTAT_LSB    0
#define INTC_FIQSTAT_WIDTH  ((uint32_t)0x00000020)

#define INTC_FIQSTAT_RST    0x0

__INLINE uint32_t intc_fiqstatus_fiqstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_FIQSTATUS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief FIQRAWSTATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           FIQRAWSTAT   0x0
 * </pre>
 */
#define INTC_FIQRAWSTATUS_ADDR   0x10001104
#define INTC_FIQRAWSTATUS_OFFSET 0x00000104
#define INTC_FIQRAWSTATUS_INDEX  0x00000041
#define INTC_FIQRAWSTATUS_RESET  0x00000000

__INLINE uint32_t intc_fiqrawstatus_get(void)
{
    return REG_PL_RD(INTC_FIQRAWSTATUS_ADDR);
}

// field definitions
#define INTC_FIQRAWSTAT_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_FIQRAWSTAT_LSB    0
#define INTC_FIQRAWSTAT_WIDTH  ((uint32_t)0x00000020)

#define INTC_FIQRAWSTAT_RST    0x0

__INLINE uint32_t intc_fiqrawstatus_fiqrawstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_FIQRAWSTATUS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief FIQENABLESET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         FIQENABLESET   0x0
 * </pre>
 */
#define INTC_FIQENABLESET_ADDR   0x10001108
#define INTC_FIQENABLESET_OFFSET 0x00000108
#define INTC_FIQENABLESET_INDEX  0x00000042
#define INTC_FIQENABLESET_RESET  0x00000000

__INLINE uint32_t intc_fiqenableset_get(void)
{
    return REG_PL_RD(INTC_FIQENABLESET_ADDR);
}

__INLINE void intc_fiqenableset_set(uint32_t value)
{
    REG_PL_WR(INTC_FIQENABLESET_ADDR, value);
}

// field definitions
#define INTC_FIQENABLESET_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_FIQENABLESET_LSB    0
#define INTC_FIQENABLESET_WIDTH  ((uint32_t)0x00000020)

#define INTC_FIQENABLESET_RST    0x0

__INLINE uint32_t intc_fiqenableset_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_FIQENABLESET_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void intc_fiqenableset_setf(uint32_t fiqenableset)
{
    ASSERT_ERR((((uint32_t)fiqenableset << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(INTC_FIQENABLESET_ADDR, (uint32_t)fiqenableset << 0);
}

/**
 * @brief FIQENABLECLEAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00       FIQENABLECLEAR   0x0
 * </pre>
 */
#define INTC_FIQENABLECLEAR_ADDR   0x1000110C
#define INTC_FIQENABLECLEAR_OFFSET 0x0000010C
#define INTC_FIQENABLECLEAR_INDEX  0x00000043
#define INTC_FIQENABLECLEAR_RESET  0x00000000

__INLINE void intc_fiqenableclear_set(uint32_t value)
{
    REG_PL_WR(INTC_FIQENABLECLEAR_ADDR, value);
}

// field definitions
#define INTC_FIQENABLECLEAR_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_FIQENABLECLEAR_LSB    0
#define INTC_FIQENABLECLEAR_WIDTH  ((uint32_t)0x00000020)

#define INTC_FIQENABLECLEAR_RST    0x0

__INLINE void intc_fiqenableclear_setf(uint32_t fiqenableclear)
{
    ASSERT_ERR((((uint32_t)fiqenableclear << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(INTC_FIQENABLECLEAR_ADDR, (uint32_t)fiqenableclear << 0);
}

/**
 * @brief FIQACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               FIQACK   0x0
 * </pre>
 */
#define INTC_FIQACK_ADDR   0x10001110
#define INTC_FIQACK_OFFSET 0x00000110
#define INTC_FIQACK_INDEX  0x00000044
#define INTC_FIQACK_RESET  0x00000000

__INLINE void intc_fiqack_set(uint32_t value)
{
    REG_PL_WR(INTC_FIQACK_ADDR, value);
}

// field definitions
#define INTC_FIQACK_MASK   ((uint32_t)0xFFFFFFFF)
#define INTC_FIQACK_LSB    0
#define INTC_FIQACK_WIDTH  ((uint32_t)0x00000020)

#define INTC_FIQACK_RST    0x0

__INLINE void intc_fiqack_setf(uint32_t fiqack)
{
    ASSERT_ERR((((uint32_t)fiqack << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(INTC_FIQACK_ADDR, (uint32_t)fiqack << 0);
}

/**
 * @brief FIQINDEX register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  04:00             FIQINDEX   0x0
 * </pre>
 */
#define INTC_FIQINDEX_ADDR   0x10001114
#define INTC_FIQINDEX_OFFSET 0x00000114
#define INTC_FIQINDEX_INDEX  0x00000045
#define INTC_FIQINDEX_RESET  0x00000000

__INLINE uint32_t intc_fiqindex_get(void)
{
    return REG_PL_RD(INTC_FIQINDEX_ADDR);
}

__INLINE void intc_fiqindex_set(uint32_t value)
{
    REG_PL_WR(INTC_FIQINDEX_ADDR, value);
}

// field definitions
#define INTC_FIQINDEX_MASK   ((uint32_t)0x0000001F)
#define INTC_FIQINDEX_LSB    0
#define INTC_FIQINDEX_WIDTH  ((uint32_t)0x00000005)

#define INTC_FIQINDEX_RST    0x0

__INLINE uint8_t intc_fiqindex_getf(void)
{
    uint32_t localVal = REG_PL_RD(INTC_FIQINDEX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000001F)) == 0);
    return (localVal >> 0);
}


#endif // _REG_INTC_H_

