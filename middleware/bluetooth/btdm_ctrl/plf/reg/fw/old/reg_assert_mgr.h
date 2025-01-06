#ifndef _REG_ASSERT_MGR_H_
#define _REG_ASSERT_MGR_H_

#include <stdint.h>
#include "_reg_assert_mgr.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_ASSERT_MGR_COUNT 4

#define REG_ASSERT_MGR_DECODING_MASK 0x0000000F

/**
 * @brief ASSERT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08             txt_size   0x0
 *     02               params   0
 *     01                 warn   0
 *     00                trigg   0
 * </pre>
 */
#define ASRT_ASSERT_ADDR   0x1000D000
#define ASRT_ASSERT_OFFSET 0x00000000
#define ASRT_ASSERT_INDEX  0x00000000
#define ASRT_ASSERT_RESET  0x00000000

__INLINE uint32_t asrt_assert_get(void)
{
    return REG_PL_RD(ASRT_ASSERT_ADDR);
}

__INLINE void asrt_assert_set(uint32_t value)
{
    REG_PL_WR(ASRT_ASSERT_ADDR, value);
}

// field definitions
#define ASRT_TXT_SIZE_MASK   ((uint32_t)0x0000FF00)
#define ASRT_TXT_SIZE_LSB    8
#define ASRT_TXT_SIZE_WIDTH  ((uint32_t)0x00000008)
#define ASRT_PARAMS_BIT      ((uint32_t)0x00000004)
#define ASRT_PARAMS_POS      2
#define ASRT_WARN_BIT        ((uint32_t)0x00000002)
#define ASRT_WARN_POS        1
#define ASRT_TRIGG_BIT       ((uint32_t)0x00000001)
#define ASRT_TRIGG_POS       0

#define ASRT_TXT_SIZE_RST    0x0
#define ASRT_PARAMS_RST      0x0
#define ASRT_WARN_RST        0x0
#define ASRT_TRIGG_RST       0x0

__INLINE void asrt_assert_pack(uint8_t txtsize, uint8_t params, uint8_t warn, uint8_t trigg)
{
    ASSERT_ERR((((uint32_t)txtsize << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)params << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)warn << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)trigg << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(ASRT_ASSERT_ADDR, ((uint32_t)txtsize << 8) | ((uint32_t)params << 2) | ((uint32_t)warn << 1) | ((uint32_t)trigg << 0));
}

__INLINE void asrt_assert_unpack(uint8_t *txtsize, uint8_t *params, uint8_t *warn, uint8_t *trigg)
{
    uint32_t localVal = REG_PL_RD(ASRT_ASSERT_ADDR);

    *txtsize = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *params = (localVal & ((uint32_t)0x00000004)) >> 2;
    *warn = (localVal & ((uint32_t)0x00000002)) >> 1;
    *trigg = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t asrt_txt_size_getf(void)
{
    uint32_t localVal = REG_PL_RD(ASRT_ASSERT_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void asrt_txt_size_setf(uint8_t txtsize)
{
    ASSERT_ERR((((uint32_t)txtsize << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(ASRT_ASSERT_ADDR, (REG_PL_RD(ASRT_ASSERT_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)txtsize << 8));
}

__INLINE uint8_t asrt_params_getf(void)
{
    uint32_t localVal = REG_PL_RD(ASRT_ASSERT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void asrt_params_setf(uint8_t params)
{
    ASSERT_ERR((((uint32_t)params << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(ASRT_ASSERT_ADDR, (REG_PL_RD(ASRT_ASSERT_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)params << 2));
}

__INLINE void asrt_warn_setf(uint8_t warn)
{
    ASSERT_ERR((((uint32_t)warn << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(ASRT_ASSERT_ADDR, (REG_PL_RD(ASRT_ASSERT_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)warn << 1));
}

__INLINE void asrt_trigg_setf(uint8_t trigg)
{
    ASSERT_ERR((((uint32_t)trigg << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(ASRT_ASSERT_ADDR, (REG_PL_RD(ASRT_ASSERT_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)trigg << 0));
}

/**
 * @brief PARAM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16               param2   0x0
 *  15:00               param1   0x0
 * </pre>
 */
#define ASRT_PARAM_ADDR   0x1000D004
#define ASRT_PARAM_OFFSET 0x00000004
#define ASRT_PARAM_INDEX  0x00000001
#define ASRT_PARAM_RESET  0x00000000

__INLINE uint32_t asrt_param_get(void)
{
    return REG_PL_RD(ASRT_PARAM_ADDR);
}

__INLINE void asrt_param_set(uint32_t value)
{
    REG_PL_WR(ASRT_PARAM_ADDR, value);
}

// field definitions
#define ASRT_PARAM_2_MASK   ((uint32_t)0xFFFF0000)
#define ASRT_PARAM_2_LSB    16
#define ASRT_PARAM_2_WIDTH  ((uint32_t)0x00000010)
#define ASRT_PARAM_1_MASK   ((uint32_t)0x0000FFFF)
#define ASRT_PARAM_1_LSB    0
#define ASRT_PARAM_1_WIDTH  ((uint32_t)0x00000010)

#define ASRT_PARAM_2_RST    0x0
#define ASRT_PARAM_1_RST    0x0

__INLINE void asrt_param_pack(uint16_t param2, uint16_t param1)
{
    ASSERT_ERR((((uint32_t)param2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)param1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(ASRT_PARAM_ADDR, ((uint32_t)param2 << 16) | ((uint32_t)param1 << 0));
}

__INLINE void asrt_param_unpack(uint16_t *param2, uint16_t *param1)
{
    uint32_t localVal = REG_PL_RD(ASRT_PARAM_ADDR);

    *param2 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *param1 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t asrt_param_2_getf(void)
{
    uint32_t localVal = REG_PL_RD(ASRT_PARAM_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void asrt_param_2_setf(uint16_t param2)
{
    ASSERT_ERR((((uint32_t)param2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_PL_WR(ASRT_PARAM_ADDR, (REG_PL_RD(ASRT_PARAM_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)param2 << 16));
}

__INLINE uint16_t asrt_param_1_getf(void)
{
    uint32_t localVal = REG_PL_RD(ASRT_PARAM_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void asrt_param_1_setf(uint16_t param1)
{
    ASSERT_ERR((((uint32_t)param1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(ASRT_PARAM_ADDR, (REG_PL_RD(ASRT_PARAM_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)param1 << 0));
}

/**
 * @brief LINE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                 line   0x0
 * </pre>
 */
#define ASRT_LINE_ADDR   0x1000D008
#define ASRT_LINE_OFFSET 0x00000008
#define ASRT_LINE_INDEX  0x00000002
#define ASRT_LINE_RESET  0x00000000

__INLINE uint32_t asrt_line_get(void)
{
    return REG_PL_RD(ASRT_LINE_ADDR);
}

__INLINE void asrt_line_set(uint32_t value)
{
    REG_PL_WR(ASRT_LINE_ADDR, value);
}

// field definitions
#define ASRT_LINE_MASK   ((uint32_t)0xFFFFFFFF)
#define ASRT_LINE_LSB    0
#define ASRT_LINE_WIDTH  ((uint32_t)0x00000020)

#define ASRT_LINE_RST    0x0

__INLINE uint32_t asrt_line_getf(void)
{
    uint32_t localVal = REG_PL_RD(ASRT_LINE_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void asrt_line_setf(uint32_t line)
{
    ASSERT_ERR((((uint32_t)line << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(ASRT_LINE_ADDR, (uint32_t)line << 0);
}

/**
 * @brief TEXT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                 addr   0x0
 * </pre>
 */
#define ASRT_TEXT_ADDR   0x1000D00C
#define ASRT_TEXT_OFFSET 0x0000000C
#define ASRT_TEXT_INDEX  0x00000003
#define ASRT_TEXT_RESET  0x00000000

__INLINE uint32_t asrt_text_get(void)
{
    return REG_PL_RD(ASRT_TEXT_ADDR);
}

__INLINE void asrt_text_set(uint32_t value)
{
    REG_PL_WR(ASRT_TEXT_ADDR, value);
}

// field definitions
#define ASRT_ADDR_MASK   ((uint32_t)0xFFFFFFFF)
#define ASRT_ADDR_LSB    0
#define ASRT_ADDR_WIDTH  ((uint32_t)0x00000020)

#define ASRT_ADDR_RST    0x0

__INLINE uint32_t asrt_addr_getf(void)
{
    uint32_t localVal = REG_PL_RD(ASRT_TEXT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void asrt_addr_setf(uint32_t addr)
{
    ASSERT_ERR((((uint32_t)addr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(ASRT_TEXT_ADDR, (uint32_t)addr << 0);
}


#endif // _REG_ASSERT_MGR_H_

