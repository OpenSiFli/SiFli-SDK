#ifndef _REG_ECC_ACCEL_H_
#define _REG_ECC_ACCEL_H_

#include <stdint.h>
#include "_reg_ecc_accel.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_ECC_ACCEL_COUNT 5

#define REG_ECC_ACCEL_DECODING_MASK 0x0000001F

/**
 * @brief CTRL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00                Start   0
 * </pre>
 */
#define ECC_CTRL_ADDR   0x1000F000
#define ECC_CTRL_OFFSET 0x00000000
#define ECC_CTRL_INDEX  0x00000000
#define ECC_CTRL_RESET  0x00000000

__INLINE uint32_t ecc_ctrl_get(void)
{
    return REG_PL_RD(ECC_CTRL_ADDR);
}

__INLINE void ecc_ctrl_set(uint32_t value)
{
    REG_PL_WR(ECC_CTRL_ADDR, value);
}

// field definitions
#define ECC_START_BIT    ((uint32_t)0x00000001)
#define ECC_START_POS    0

#define ECC_START_RST    0x0

__INLINE void ecc_start_setf(uint8_t start)
{
    ASSERT_ERR((((uint32_t)start << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(ECC_CTRL_ADDR, (uint32_t)start << 0);
}

/**
 * @brief PRIV_PTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             priv_ptr   0x0
 * </pre>
 */
#define ECC_PRIV_PTR_ADDR   0x1000F004
#define ECC_PRIV_PTR_OFFSET 0x00000004
#define ECC_PRIV_PTR_INDEX  0x00000001
#define ECC_PRIV_PTR_RESET  0x00000000

__INLINE uint32_t ecc_priv_ptr_get(void)
{
    return REG_PL_RD(ECC_PRIV_PTR_ADDR);
}

__INLINE void ecc_priv_ptr_set(uint32_t value)
{
    REG_PL_WR(ECC_PRIV_PTR_ADDR, value);
}

// field definitions
#define ECC_PRIV_PTR_MASK   ((uint32_t)0xFFFFFFFF)
#define ECC_PRIV_PTR_LSB    0
#define ECC_PRIV_PTR_WIDTH  ((uint32_t)0x00000020)

#define ECC_PRIV_PTR_RST    0x0

__INLINE uint32_t ecc_priv_ptr_getf(void)
{
    uint32_t localVal = REG_PL_RD(ECC_PRIV_PTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ecc_priv_ptr_setf(uint32_t privptr)
{
    ASSERT_ERR((((uint32_t)privptr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(ECC_PRIV_PTR_ADDR, (uint32_t)privptr << 0);
}

/**
 * @brief PUB_X_PTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            pub_x_ptr   0x0
 * </pre>
 */
#define ECC_PUB_X_PTR_ADDR   0x1000F008
#define ECC_PUB_X_PTR_OFFSET 0x00000008
#define ECC_PUB_X_PTR_INDEX  0x00000002
#define ECC_PUB_X_PTR_RESET  0x00000000

__INLINE uint32_t ecc_pub_x_ptr_get(void)
{
    return REG_PL_RD(ECC_PUB_X_PTR_ADDR);
}

__INLINE void ecc_pub_x_ptr_set(uint32_t value)
{
    REG_PL_WR(ECC_PUB_X_PTR_ADDR, value);
}

// field definitions
#define ECC_PUB_X_PTR_MASK   ((uint32_t)0xFFFFFFFF)
#define ECC_PUB_X_PTR_LSB    0
#define ECC_PUB_X_PTR_WIDTH  ((uint32_t)0x00000020)

#define ECC_PUB_X_PTR_RST    0x0

__INLINE uint32_t ecc_pub_x_ptr_getf(void)
{
    uint32_t localVal = REG_PL_RD(ECC_PUB_X_PTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ecc_pub_x_ptr_setf(uint32_t pubxptr)
{
    ASSERT_ERR((((uint32_t)pubxptr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(ECC_PUB_X_PTR_ADDR, (uint32_t)pubxptr << 0);
}

/**
 * @brief PUB_Y_PTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            pub_y_ptr   0x0
 * </pre>
 */
#define ECC_PUB_Y_PTR_ADDR   0x1000F00C
#define ECC_PUB_Y_PTR_OFFSET 0x0000000C
#define ECC_PUB_Y_PTR_INDEX  0x00000003
#define ECC_PUB_Y_PTR_RESET  0x00000000

__INLINE uint32_t ecc_pub_y_ptr_get(void)
{
    return REG_PL_RD(ECC_PUB_Y_PTR_ADDR);
}

__INLINE void ecc_pub_y_ptr_set(uint32_t value)
{
    REG_PL_WR(ECC_PUB_Y_PTR_ADDR, value);
}

// field definitions
#define ECC_PUB_Y_PTR_MASK   ((uint32_t)0xFFFFFFFF)
#define ECC_PUB_Y_PTR_LSB    0
#define ECC_PUB_Y_PTR_WIDTH  ((uint32_t)0x00000020)

#define ECC_PUB_Y_PTR_RST    0x0

__INLINE uint32_t ecc_pub_y_ptr_getf(void)
{
    uint32_t localVal = REG_PL_RD(ECC_PUB_Y_PTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ecc_pub_y_ptr_setf(uint32_t pubyptr)
{
    ASSERT_ERR((((uint32_t)pubyptr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(ECC_PUB_Y_PTR_ADDR, (uint32_t)pubyptr << 0);
}

/**
 * @brief RESULT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               RESULT   0x0
 * </pre>
 */
#define ECC_RESULT_ADDR   0x1000F010
#define ECC_RESULT_OFFSET 0x00000010
#define ECC_RESULT_INDEX  0x00000004
#define ECC_RESULT_RESET  0x00000000

__INLINE uint32_t ecc_result_get(void)
{
    return REG_PL_RD(ECC_RESULT_ADDR);
}

__INLINE void ecc_result_set(uint32_t value)
{
    REG_PL_WR(ECC_RESULT_ADDR, value);
}

// field definitions
#define ECC_RESULT_MASK   ((uint32_t)0xFFFFFFFF)
#define ECC_RESULT_LSB    0
#define ECC_RESULT_WIDTH  ((uint32_t)0x00000020)

#define ECC_RESULT_RST    0x0

__INLINE uint32_t ecc_result_getf(void)
{
    uint32_t localVal = REG_PL_RD(ECC_RESULT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ecc_result_setf(uint32_t result)
{
    ASSERT_ERR((((uint32_t)result << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(ECC_RESULT_ADDR, (uint32_t)result << 0);
}


#endif // _REG_ECC_ACCEL_H_

