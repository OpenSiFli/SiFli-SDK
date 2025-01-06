#ifndef _REG_TIMER_H_
#define _REG_TIMER_H_

#include <stdint.h>
#include "_reg_timer.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_TIMER_COUNT 7

#define REG_TIMER_DECODING_MASK 0x0000001F

/**
 * @brief TIMER_CTRL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31               enable   0
 *     30                reset   0
 *  27:00           unit_in_us   0x3E8
 * </pre>
 */
#define TMR_TIMER_CTRL_ADDR   0x1000E000
#define TMR_TIMER_CTRL_OFFSET 0x00000000
#define TMR_TIMER_CTRL_INDEX  0x00000000
#define TMR_TIMER_CTRL_RESET  0x000003E8

__INLINE uint32_t tmr_timer_ctrl_get(void)
{
    return REG_PL_RD(TMR_TIMER_CTRL_ADDR);
}

__INLINE void tmr_timer_ctrl_set(uint32_t value)
{
    REG_PL_WR(TMR_TIMER_CTRL_ADDR, value);
}

// field definitions
#define TMR_ENABLE_BIT        ((uint32_t)0x80000000)
#define TMR_ENABLE_POS        31
#define TMR_RESET_BIT         ((uint32_t)0x40000000)
#define TMR_RESET_POS         30
#define TMR_UNIT_IN_US_MASK   ((uint32_t)0x0FFFFFFF)
#define TMR_UNIT_IN_US_LSB    0
#define TMR_UNIT_IN_US_WIDTH  ((uint32_t)0x0000001C)

#define TMR_ENABLE_RST        0x0
#define TMR_RESET_RST         0x0
#define TMR_UNIT_IN_US_RST    0x3E8

__INLINE void tmr_timer_ctrl_pack(uint8_t enable, uint8_t reset, uint32_t unitinus)
{
    ASSERT_ERR((((uint32_t)enable << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)reset << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)unitinus << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_PL_WR(TMR_TIMER_CTRL_ADDR, ((uint32_t)enable << 31) | ((uint32_t)reset << 30) | ((uint32_t)unitinus << 0));
}

__INLINE void tmr_timer_ctrl_unpack(uint8_t *enable, uint8_t *reset, uint32_t *unitinus)
{
    uint32_t localVal = REG_PL_RD(TMR_TIMER_CTRL_ADDR);

    *enable = (localVal & ((uint32_t)0x80000000)) >> 31;
    *reset = (localVal & ((uint32_t)0x40000000)) >> 30;
    *unitinus = (localVal & ((uint32_t)0x0FFFFFFF)) >> 0;
}

__INLINE uint8_t tmr_enable_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_TIMER_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void tmr_enable_setf(uint8_t enable)
{
    ASSERT_ERR((((uint32_t)enable << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(TMR_TIMER_CTRL_ADDR, (REG_PL_RD(TMR_TIMER_CTRL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)enable << 31));
}

__INLINE uint8_t tmr_reset_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_TIMER_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void tmr_reset_setf(uint8_t reset)
{
    ASSERT_ERR((((uint32_t)reset << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_PL_WR(TMR_TIMER_CTRL_ADDR, (REG_PL_RD(TMR_TIMER_CTRL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)reset << 30));
}

__INLINE uint32_t tmr_unit_in_us_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_TIMER_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x0FFFFFFF)) >> 0);
}

__INLINE void tmr_unit_in_us_setf(uint32_t unitinus)
{
    ASSERT_ERR((((uint32_t)unitinus << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_PL_WR(TMR_TIMER_CTRL_ADDR, (REG_PL_RD(TMR_TIMER_CTRL_ADDR) & ~((uint32_t)0x0FFFFFFF)) | ((uint32_t)unitinus << 0));
}

/**
 * @brief CURRENT_TIME register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00                 time   0x0
 * </pre>
 */
#define TMR_CURRENT_TIME_ADDR   0x1000E004
#define TMR_CURRENT_TIME_OFFSET 0x00000004
#define TMR_CURRENT_TIME_INDEX  0x00000001
#define TMR_CURRENT_TIME_RESET  0x00000000

__INLINE uint32_t tmr_current_time_get(void)
{
    return REG_PL_RD(TMR_CURRENT_TIME_ADDR);
}

__INLINE void tmr_current_time_set(uint32_t value)
{
    REG_PL_WR(TMR_CURRENT_TIME_ADDR, value);
}

// field definitions
#define TMR_TIME_MASK   ((uint32_t)0x0FFFFFFF)
#define TMR_TIME_LSB    0
#define TMR_TIME_WIDTH  ((uint32_t)0x0000001C)

#define TMR_TIME_RST    0x0

__INLINE uint32_t tmr_time_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_CURRENT_TIME_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief END_TIME register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00             end_time   0x0
 * </pre>
 */
#define TMR_END_TIME_ADDR   0x1000E008
#define TMR_END_TIME_OFFSET 0x00000008
#define TMR_END_TIME_INDEX  0x00000002
#define TMR_END_TIME_RESET  0x00000000

__INLINE uint32_t tmr_end_time_get(void)
{
    return REG_PL_RD(TMR_END_TIME_ADDR);
}

__INLINE void tmr_end_time_set(uint32_t value)
{
    REG_PL_WR(TMR_END_TIME_ADDR, value);
}

// field definitions
#define TMR_END_TIME_MASK   ((uint32_t)0x0FFFFFFF)
#define TMR_END_TIME_LSB    0
#define TMR_END_TIME_WIDTH  ((uint32_t)0x0000001C)

#define TMR_END_TIME_RST    0x0

__INLINE uint32_t tmr_end_time_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_END_TIME_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void tmr_end_time_setf(uint32_t endtime)
{
    ASSERT_ERR((((uint32_t)endtime << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_PL_WR(TMR_END_TIME_ADDR, (uint32_t)endtime << 0);
}

/**
 * @brief PERIODIC_TIME register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00               period   0x0
 * </pre>
 */
#define TMR_PERIODIC_TIME_ADDR   0x1000E00C
#define TMR_PERIODIC_TIME_OFFSET 0x0000000C
#define TMR_PERIODIC_TIME_INDEX  0x00000003
#define TMR_PERIODIC_TIME_RESET  0x00000000

__INLINE uint32_t tmr_periodic_time_get(void)
{
    return REG_PL_RD(TMR_PERIODIC_TIME_ADDR);
}

__INLINE void tmr_periodic_time_set(uint32_t value)
{
    REG_PL_WR(TMR_PERIODIC_TIME_ADDR, value);
}

// field definitions
#define TMR_PERIOD_MASK   ((uint32_t)0x0FFFFFFF)
#define TMR_PERIOD_LSB    0
#define TMR_PERIOD_WIDTH  ((uint32_t)0x0000001C)

#define TMR_PERIOD_RST    0x0

__INLINE uint32_t tmr_period_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_PERIODIC_TIME_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void tmr_period_setf(uint32_t period)
{
    ASSERT_ERR((((uint32_t)period << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_PL_WR(TMR_PERIODIC_TIME_ADDR, (uint32_t)period << 0);
}

/**
 * @brief ISR_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01        periodic_stat   0
 *     00        end_time_stat   0
 * </pre>
 */
#define TMR_ISR_STATUS_ADDR   0x1000E010
#define TMR_ISR_STATUS_OFFSET 0x00000010
#define TMR_ISR_STATUS_INDEX  0x00000004
#define TMR_ISR_STATUS_RESET  0x00000000

__INLINE uint32_t tmr_isr_status_get(void)
{
    return REG_PL_RD(TMR_ISR_STATUS_ADDR);
}

// field definitions
#define TMR_PERIODIC_STAT_BIT    ((uint32_t)0x00000002)
#define TMR_PERIODIC_STAT_POS    1
#define TMR_END_TIME_STAT_BIT    ((uint32_t)0x00000001)
#define TMR_END_TIME_STAT_POS    0

#define TMR_PERIODIC_STAT_RST    0x0
#define TMR_END_TIME_STAT_RST    0x0

__INLINE void tmr_isr_status_unpack(uint8_t *periodicstat, uint8_t *endtimestat)
{
    uint32_t localVal = REG_PL_RD(TMR_ISR_STATUS_ADDR);

    *periodicstat = (localVal & ((uint32_t)0x00000002)) >> 1;
    *endtimestat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t tmr_periodic_stat_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_ISR_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t tmr_end_time_stat_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_ISR_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief ISR_EN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01          periodic_en   0
 *     00          end_time_en   0
 * </pre>
 */
#define TMR_ISR_EN_ADDR   0x1000E014
#define TMR_ISR_EN_OFFSET 0x00000014
#define TMR_ISR_EN_INDEX  0x00000005
#define TMR_ISR_EN_RESET  0x00000000

__INLINE uint32_t tmr_isr_en_get(void)
{
    return REG_PL_RD(TMR_ISR_EN_ADDR);
}

__INLINE void tmr_isr_en_set(uint32_t value)
{
    REG_PL_WR(TMR_ISR_EN_ADDR, value);
}

// field definitions
#define TMR_PERIODIC_EN_BIT    ((uint32_t)0x00000002)
#define TMR_PERIODIC_EN_POS    1
#define TMR_END_TIME_EN_BIT    ((uint32_t)0x00000001)
#define TMR_END_TIME_EN_POS    0

#define TMR_PERIODIC_EN_RST    0x0
#define TMR_END_TIME_EN_RST    0x0

__INLINE void tmr_isr_en_pack(uint8_t periodicen, uint8_t endtimeen)
{
    ASSERT_ERR((((uint32_t)periodicen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)endtimeen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(TMR_ISR_EN_ADDR, ((uint32_t)periodicen << 1) | ((uint32_t)endtimeen << 0));
}

__INLINE void tmr_isr_en_unpack(uint8_t *periodicen, uint8_t *endtimeen)
{
    uint32_t localVal = REG_PL_RD(TMR_ISR_EN_ADDR);

    *periodicen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *endtimeen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t tmr_periodic_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_ISR_EN_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void tmr_periodic_en_setf(uint8_t periodicen)
{
    ASSERT_ERR((((uint32_t)periodicen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(TMR_ISR_EN_ADDR, (REG_PL_RD(TMR_ISR_EN_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)periodicen << 1));
}

__INLINE uint8_t tmr_end_time_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(TMR_ISR_EN_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void tmr_end_time_en_setf(uint8_t endtimeen)
{
    ASSERT_ERR((((uint32_t)endtimeen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(TMR_ISR_EN_ADDR, (REG_PL_RD(TMR_ISR_EN_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)endtimeen << 0));
}

/**
 * @brief ISR_CLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01         periodic_clr   0
 *     00         end_time_clr   0
 * </pre>
 */
#define TMR_ISR_CLR_ADDR   0x1000E018
#define TMR_ISR_CLR_OFFSET 0x00000018
#define TMR_ISR_CLR_INDEX  0x00000006
#define TMR_ISR_CLR_RESET  0x00000000

__INLINE uint32_t tmr_isr_clr_get(void)
{
    return REG_PL_RD(TMR_ISR_CLR_ADDR);
}

__INLINE void tmr_isr_clr_set(uint32_t value)
{
    REG_PL_WR(TMR_ISR_CLR_ADDR, value);
}

// field definitions
#define TMR_PERIODIC_CLR_BIT    ((uint32_t)0x00000002)
#define TMR_PERIODIC_CLR_POS    1
#define TMR_END_TIME_CLR_BIT    ((uint32_t)0x00000001)
#define TMR_END_TIME_CLR_POS    0

#define TMR_PERIODIC_CLR_RST    0x0
#define TMR_END_TIME_CLR_RST    0x0

__INLINE void tmr_isr_clr_pack(uint8_t periodicclr, uint8_t endtimeclr)
{
    ASSERT_ERR((((uint32_t)periodicclr << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)endtimeclr << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(TMR_ISR_CLR_ADDR, ((uint32_t)periodicclr << 1) | ((uint32_t)endtimeclr << 0));
}

__INLINE void tmr_isr_clr_unpack(uint8_t *periodicclr, uint8_t *endtimeclr)
{
    uint32_t localVal = REG_PL_RD(TMR_ISR_CLR_ADDR);

    *periodicclr = (localVal & ((uint32_t)0x00000002)) >> 1;
    *endtimeclr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE void tmr_periodic_clr_setf(uint8_t periodicclr)
{
    ASSERT_ERR((((uint32_t)periodicclr << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(TMR_ISR_CLR_ADDR, (REG_PL_RD(TMR_ISR_CLR_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)periodicclr << 1));
}

__INLINE void tmr_end_time_clr_setf(uint8_t endtimeclr)
{
    ASSERT_ERR((((uint32_t)endtimeclr << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(TMR_ISR_CLR_ADDR, (REG_PL_RD(TMR_ISR_CLR_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)endtimeclr << 0));
}


#endif // _REG_TIMER_H_

