#ifndef _REG_IQGEN_H_
#define _REG_IQGEN_H_

#include <stdint.h>
#include "_reg_iqgen.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_IQGEN_COUNT 6

#define REG_IQGEN_DECODING_MASK 0x0000001F

/**
 * @brief DFGENCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            DF_SOURCE   0
 *  25:16       AOD_SWITCH_DLY   0x0
 *  11:08       IQ_INVALID_DLY   0x0
 *  06:04           NB_ANTENNA   0x0
 *     03         PATTERN_MODE   0
 *     02          TIMING_MODE   0
 *     01       IQ_SAMPLING_EN   0
 *     00    ANTENNA_SWITCH_EN   0
 * </pre>
 */
#define IQGEN_DFGENCNTL_ADDR   0x1000A000
#define IQGEN_DFGENCNTL_OFFSET 0x00000000
#define IQGEN_DFGENCNTL_INDEX  0x00000000
#define IQGEN_DFGENCNTL_RESET  0x00000000

__INLINE uint32_t iqgen_dfgencntl_get(void)
{
    return REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
}

__INLINE void iqgen_dfgencntl_set(uint32_t value)
{
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, value);
}

// field definitions
#define IQGEN_DF_SOURCE_BIT            ((uint32_t)0x80000000)
#define IQGEN_DF_SOURCE_POS            31
#define IQGEN_AOD_SWITCH_DLY_MASK      ((uint32_t)0x03FF0000)
#define IQGEN_AOD_SWITCH_DLY_LSB       16
#define IQGEN_AOD_SWITCH_DLY_WIDTH     ((uint32_t)0x0000000A)
#define IQGEN_IQ_INVALID_DLY_MASK      ((uint32_t)0x00000F00)
#define IQGEN_IQ_INVALID_DLY_LSB       8
#define IQGEN_IQ_INVALID_DLY_WIDTH     ((uint32_t)0x00000004)
#define IQGEN_NB_ANTENNA_MASK          ((uint32_t)0x00000070)
#define IQGEN_NB_ANTENNA_LSB           4
#define IQGEN_NB_ANTENNA_WIDTH         ((uint32_t)0x00000003)
#define IQGEN_PATTERN_MODE_BIT         ((uint32_t)0x00000008)
#define IQGEN_PATTERN_MODE_POS         3
#define IQGEN_TIMING_MODE_BIT          ((uint32_t)0x00000004)
#define IQGEN_TIMING_MODE_POS          2
#define IQGEN_IQ_SAMPLING_EN_BIT       ((uint32_t)0x00000002)
#define IQGEN_IQ_SAMPLING_EN_POS       1
#define IQGEN_ANTENNA_SWITCH_EN_BIT    ((uint32_t)0x00000001)
#define IQGEN_ANTENNA_SWITCH_EN_POS    0

#define IQGEN_DF_SOURCE_RST            0x0
#define IQGEN_AOD_SWITCH_DLY_RST       0x0
#define IQGEN_IQ_INVALID_DLY_RST       0x0
#define IQGEN_NB_ANTENNA_RST           0x0
#define IQGEN_PATTERN_MODE_RST         0x0
#define IQGEN_TIMING_MODE_RST          0x0
#define IQGEN_IQ_SAMPLING_EN_RST       0x0
#define IQGEN_ANTENNA_SWITCH_EN_RST    0x0

__INLINE void iqgen_dfgencntl_pack(uint8_t dfsource, uint16_t aodswitchdly, uint8_t iqinvaliddly, uint8_t nbantenna, uint8_t patternmode, uint8_t timingmode, uint8_t iqsamplingen, uint8_t antennaswitchen)
{
    ASSERT_ERR((((uint32_t)dfsource << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)aodswitchdly << 16) & ~((uint32_t)0x03FF0000)) == 0);
    ASSERT_ERR((((uint32_t)iqinvaliddly << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)nbantenna << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)patternmode << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)timingmode << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)iqsamplingen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)antennaswitchen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, ((uint32_t)dfsource << 31) | ((uint32_t)aodswitchdly << 16) | ((uint32_t)iqinvaliddly << 8) | ((uint32_t)nbantenna << 4) | ((uint32_t)patternmode << 3) | ((uint32_t)timingmode << 2) | ((uint32_t)iqsamplingen << 1) | ((uint32_t)antennaswitchen << 0));
}

__INLINE void iqgen_dfgencntl_unpack(uint8_t *dfsource, uint16_t *aodswitchdly, uint8_t *iqinvaliddly, uint8_t *nbantenna, uint8_t *patternmode, uint8_t *timingmode, uint8_t *iqsamplingen, uint8_t *antennaswitchen)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);

    *dfsource = (localVal & ((uint32_t)0x80000000)) >> 31;
    *aodswitchdly = (localVal & ((uint32_t)0x03FF0000)) >> 16;
    *iqinvaliddly = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *nbantenna = (localVal & ((uint32_t)0x00000070)) >> 4;
    *patternmode = (localVal & ((uint32_t)0x00000008)) >> 3;
    *timingmode = (localVal & ((uint32_t)0x00000004)) >> 2;
    *iqsamplingen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *antennaswitchen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t iqgen_df_source_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void iqgen_df_source_setf(uint8_t dfsource)
{
    ASSERT_ERR((((uint32_t)dfsource << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)dfsource << 31));
}

__INLINE uint16_t iqgen_aod_switch_dly_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x03FF0000)) >> 16);
}

__INLINE void iqgen_aod_switch_dly_setf(uint16_t aodswitchdly)
{
    ASSERT_ERR((((uint32_t)aodswitchdly << 16) & ~((uint32_t)0x03FF0000)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x03FF0000)) | ((uint32_t)aodswitchdly << 16));
}

__INLINE uint8_t iqgen_iq_invalid_dly_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void iqgen_iq_invalid_dly_setf(uint8_t iqinvaliddly)
{
    ASSERT_ERR((((uint32_t)iqinvaliddly << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)iqinvaliddly << 8));
}

__INLINE uint8_t iqgen_nb_antenna_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void iqgen_nb_antenna_setf(uint8_t nbantenna)
{
    ASSERT_ERR((((uint32_t)nbantenna << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)nbantenna << 4));
}

__INLINE uint8_t iqgen_pattern_mode_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void iqgen_pattern_mode_setf(uint8_t patternmode)
{
    ASSERT_ERR((((uint32_t)patternmode << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)patternmode << 3));
}

__INLINE uint8_t iqgen_timing_mode_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void iqgen_timing_mode_setf(uint8_t timingmode)
{
    ASSERT_ERR((((uint32_t)timingmode << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)timingmode << 2));
}

__INLINE uint8_t iqgen_iq_sampling_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void iqgen_iq_sampling_en_setf(uint8_t iqsamplingen)
{
    ASSERT_ERR((((uint32_t)iqsamplingen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)iqsamplingen << 1));
}

__INLINE uint8_t iqgen_antenna_switch_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_DFGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void iqgen_antenna_switch_en_setf(uint8_t antennaswitchen)
{
    ASSERT_ERR((((uint32_t)antennaswitchen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(IQGEN_DFGENCNTL_ADDR, (REG_PL_RD(IQGEN_DFGENCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)antennaswitchen << 0));
}

/**
 * @brief IQ_SAMPLE_BEH_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:30              Q_CNTL7   0x0
 *  29:28              I_CNTL7   0x0
 *  27:26              Q_CNTL6   0x0
 *  25:24              I_CNTL6   0x0
 *  23:22              Q_CNTL5   0x0
 *  21:20              I_CNTL5   0x0
 *  19:18              Q_CNTL4   0x0
 *  17:16             IQ_CNTL4   0x0
 *  15:14              Q_CNTL3   0x0
 *  13:12              I_CNTL3   0x0
 *  11:10              Q_CNTL2   0x0
 *  09:08              I_CNTL2   0x0
 *  07:06              Q_CNTL1   0x0
 *  05:04              I_CNTL1   0x0
 *  03:02              Q_CNTL0   0x0
 *  01:00              I_CNTL0   0x0
 * </pre>
 */
#define IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR   0x1000A004
#define IQGEN_IQ_SAMPLE_BEH_CNTL_OFFSET 0x00000004
#define IQGEN_IQ_SAMPLE_BEH_CNTL_INDEX  0x00000001
#define IQGEN_IQ_SAMPLE_BEH_CNTL_RESET  0x00000000

__INLINE uint32_t iqgen_iq_sample_beh_cntl_get(void)
{
    return REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
}

__INLINE void iqgen_iq_sample_beh_cntl_set(uint32_t value)
{
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, value);
}

// field definitions
#define IQGEN_Q_CNTL7_MASK    ((uint32_t)0xC0000000)
#define IQGEN_Q_CNTL7_LSB     30
#define IQGEN_Q_CNTL7_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_I_CNTL7_MASK    ((uint32_t)0x30000000)
#define IQGEN_I_CNTL7_LSB     28
#define IQGEN_I_CNTL7_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_Q_CNTL6_MASK    ((uint32_t)0x0C000000)
#define IQGEN_Q_CNTL6_LSB     26
#define IQGEN_Q_CNTL6_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_I_CNTL6_MASK    ((uint32_t)0x03000000)
#define IQGEN_I_CNTL6_LSB     24
#define IQGEN_I_CNTL6_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_Q_CNTL5_MASK    ((uint32_t)0x00C00000)
#define IQGEN_Q_CNTL5_LSB     22
#define IQGEN_Q_CNTL5_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_I_CNTL5_MASK    ((uint32_t)0x00300000)
#define IQGEN_I_CNTL5_LSB     20
#define IQGEN_I_CNTL5_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_Q_CNTL4_MASK    ((uint32_t)0x000C0000)
#define IQGEN_Q_CNTL4_LSB     18
#define IQGEN_Q_CNTL4_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_IQ_CNTL4_MASK   ((uint32_t)0x00030000)
#define IQGEN_IQ_CNTL4_LSB    16
#define IQGEN_IQ_CNTL4_WIDTH  ((uint32_t)0x00000002)
#define IQGEN_Q_CNTL3_MASK    ((uint32_t)0x0000C000)
#define IQGEN_Q_CNTL3_LSB     14
#define IQGEN_Q_CNTL3_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_I_CNTL3_MASK    ((uint32_t)0x00003000)
#define IQGEN_I_CNTL3_LSB     12
#define IQGEN_I_CNTL3_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_Q_CNTL2_MASK    ((uint32_t)0x00000C00)
#define IQGEN_Q_CNTL2_LSB     10
#define IQGEN_Q_CNTL2_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_I_CNTL2_MASK    ((uint32_t)0x00000300)
#define IQGEN_I_CNTL2_LSB     8
#define IQGEN_I_CNTL2_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_Q_CNTL1_MASK    ((uint32_t)0x000000C0)
#define IQGEN_Q_CNTL1_LSB     6
#define IQGEN_Q_CNTL1_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_I_CNTL1_MASK    ((uint32_t)0x00000030)
#define IQGEN_I_CNTL1_LSB     4
#define IQGEN_I_CNTL1_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_Q_CNTL0_MASK    ((uint32_t)0x0000000C)
#define IQGEN_Q_CNTL0_LSB     2
#define IQGEN_Q_CNTL0_WIDTH   ((uint32_t)0x00000002)
#define IQGEN_I_CNTL0_MASK    ((uint32_t)0x00000003)
#define IQGEN_I_CNTL0_LSB     0
#define IQGEN_I_CNTL0_WIDTH   ((uint32_t)0x00000002)

#define IQGEN_Q_CNTL7_RST     0x0
#define IQGEN_I_CNTL7_RST     0x0
#define IQGEN_Q_CNTL6_RST     0x0
#define IQGEN_I_CNTL6_RST     0x0
#define IQGEN_Q_CNTL5_RST     0x0
#define IQGEN_I_CNTL5_RST     0x0
#define IQGEN_Q_CNTL4_RST     0x0
#define IQGEN_IQ_CNTL4_RST    0x0
#define IQGEN_Q_CNTL3_RST     0x0
#define IQGEN_I_CNTL3_RST     0x0
#define IQGEN_Q_CNTL2_RST     0x0
#define IQGEN_I_CNTL2_RST     0x0
#define IQGEN_Q_CNTL1_RST     0x0
#define IQGEN_I_CNTL1_RST     0x0
#define IQGEN_Q_CNTL0_RST     0x0
#define IQGEN_I_CNTL0_RST     0x0

__INLINE void iqgen_iq_sample_beh_cntl_pack(uint8_t qcntl7, uint8_t icntl7, uint8_t qcntl6, uint8_t icntl6, uint8_t qcntl5, uint8_t icntl5, uint8_t qcntl4, uint8_t iqcntl4, uint8_t qcntl3, uint8_t icntl3, uint8_t qcntl2, uint8_t icntl2, uint8_t qcntl1, uint8_t icntl1, uint8_t qcntl0, uint8_t icntl0)
{
    ASSERT_ERR((((uint32_t)qcntl7 << 30) & ~((uint32_t)0xC0000000)) == 0);
    ASSERT_ERR((((uint32_t)icntl7 << 28) & ~((uint32_t)0x30000000)) == 0);
    ASSERT_ERR((((uint32_t)qcntl6 << 26) & ~((uint32_t)0x0C000000)) == 0);
    ASSERT_ERR((((uint32_t)icntl6 << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)qcntl5 << 22) & ~((uint32_t)0x00C00000)) == 0);
    ASSERT_ERR((((uint32_t)icntl5 << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)qcntl4 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)iqcntl4 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)qcntl3 << 14) & ~((uint32_t)0x0000C000)) == 0);
    ASSERT_ERR((((uint32_t)icntl3 << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)qcntl2 << 10) & ~((uint32_t)0x00000C00)) == 0);
    ASSERT_ERR((((uint32_t)icntl2 << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)qcntl1 << 6) & ~((uint32_t)0x000000C0)) == 0);
    ASSERT_ERR((((uint32_t)icntl1 << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)qcntl0 << 2) & ~((uint32_t)0x0000000C)) == 0);
    ASSERT_ERR((((uint32_t)icntl0 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, ((uint32_t)qcntl7 << 30) | ((uint32_t)icntl7 << 28) | ((uint32_t)qcntl6 << 26) | ((uint32_t)icntl6 << 24) | ((uint32_t)qcntl5 << 22) | ((uint32_t)icntl5 << 20) | ((uint32_t)qcntl4 << 18) | ((uint32_t)iqcntl4 << 16) | ((uint32_t)qcntl3 << 14) | ((uint32_t)icntl3 << 12) | ((uint32_t)qcntl2 << 10) | ((uint32_t)icntl2 << 8) | ((uint32_t)qcntl1 << 6) | ((uint32_t)icntl1 << 4) | ((uint32_t)qcntl0 << 2) | ((uint32_t)icntl0 << 0));
}

__INLINE void iqgen_iq_sample_beh_cntl_unpack(uint8_t *qcntl7, uint8_t *icntl7, uint8_t *qcntl6, uint8_t *icntl6, uint8_t *qcntl5, uint8_t *icntl5, uint8_t *qcntl4, uint8_t *iqcntl4, uint8_t *qcntl3, uint8_t *icntl3, uint8_t *qcntl2, uint8_t *icntl2, uint8_t *qcntl1, uint8_t *icntl1, uint8_t *qcntl0, uint8_t *icntl0)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);

    *qcntl7 = (localVal & ((uint32_t)0xC0000000)) >> 30;
    *icntl7 = (localVal & ((uint32_t)0x30000000)) >> 28;
    *qcntl6 = (localVal & ((uint32_t)0x0C000000)) >> 26;
    *icntl6 = (localVal & ((uint32_t)0x03000000)) >> 24;
    *qcntl5 = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *icntl5 = (localVal & ((uint32_t)0x00300000)) >> 20;
    *qcntl4 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *iqcntl4 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *qcntl3 = (localVal & ((uint32_t)0x0000C000)) >> 14;
    *icntl3 = (localVal & ((uint32_t)0x00003000)) >> 12;
    *qcntl2 = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *icntl2 = (localVal & ((uint32_t)0x00000300)) >> 8;
    *qcntl1 = (localVal & ((uint32_t)0x000000C0)) >> 6;
    *icntl1 = (localVal & ((uint32_t)0x00000030)) >> 4;
    *qcntl0 = (localVal & ((uint32_t)0x0000000C)) >> 2;
    *icntl0 = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t iqgen_q_cntl7_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0xC0000000)) >> 30);
}

__INLINE void iqgen_q_cntl7_setf(uint8_t qcntl7)
{
    ASSERT_ERR((((uint32_t)qcntl7 << 30) & ~((uint32_t)0xC0000000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0xC0000000)) | ((uint32_t)qcntl7 << 30));
}

__INLINE uint8_t iqgen_i_cntl7_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void iqgen_i_cntl7_setf(uint8_t icntl7)
{
    ASSERT_ERR((((uint32_t)icntl7 << 28) & ~((uint32_t)0x30000000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)icntl7 << 28));
}

__INLINE uint8_t iqgen_q_cntl6_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0C000000)) >> 26);
}

__INLINE void iqgen_q_cntl6_setf(uint8_t qcntl6)
{
    ASSERT_ERR((((uint32_t)qcntl6 << 26) & ~((uint32_t)0x0C000000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x0C000000)) | ((uint32_t)qcntl6 << 26));
}

__INLINE uint8_t iqgen_i_cntl6_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void iqgen_i_cntl6_setf(uint8_t icntl6)
{
    ASSERT_ERR((((uint32_t)icntl6 << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)icntl6 << 24));
}

__INLINE uint8_t iqgen_q_cntl5_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE void iqgen_q_cntl5_setf(uint8_t qcntl5)
{
    ASSERT_ERR((((uint32_t)qcntl5 << 22) & ~((uint32_t)0x00C00000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00C00000)) | ((uint32_t)qcntl5 << 22));
}

__INLINE uint8_t iqgen_i_cntl5_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void iqgen_i_cntl5_setf(uint8_t icntl5)
{
    ASSERT_ERR((((uint32_t)icntl5 << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)icntl5 << 20));
}

__INLINE uint8_t iqgen_q_cntl4_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void iqgen_q_cntl4_setf(uint8_t qcntl4)
{
    ASSERT_ERR((((uint32_t)qcntl4 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)qcntl4 << 18));
}

__INLINE uint8_t iqgen_iq_cntl4_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void iqgen_iq_cntl4_setf(uint8_t iqcntl4)
{
    ASSERT_ERR((((uint32_t)iqcntl4 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)iqcntl4 << 16));
}

__INLINE uint8_t iqgen_q_cntl3_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void iqgen_q_cntl3_setf(uint8_t qcntl3)
{
    ASSERT_ERR((((uint32_t)qcntl3 << 14) & ~((uint32_t)0x0000C000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)qcntl3 << 14));
}

__INLINE uint8_t iqgen_i_cntl3_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void iqgen_i_cntl3_setf(uint8_t icntl3)
{
    ASSERT_ERR((((uint32_t)icntl3 << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)icntl3 << 12));
}

__INLINE uint8_t iqgen_q_cntl2_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void iqgen_q_cntl2_setf(uint8_t qcntl2)
{
    ASSERT_ERR((((uint32_t)qcntl2 << 10) & ~((uint32_t)0x00000C00)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)qcntl2 << 10));
}

__INLINE uint8_t iqgen_i_cntl2_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void iqgen_i_cntl2_setf(uint8_t icntl2)
{
    ASSERT_ERR((((uint32_t)icntl2 << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)icntl2 << 8));
}

__INLINE uint8_t iqgen_q_cntl1_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000C0)) >> 6);
}

__INLINE void iqgen_q_cntl1_setf(uint8_t qcntl1)
{
    ASSERT_ERR((((uint32_t)qcntl1 << 6) & ~((uint32_t)0x000000C0)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x000000C0)) | ((uint32_t)qcntl1 << 6));
}

__INLINE uint8_t iqgen_i_cntl1_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void iqgen_i_cntl1_setf(uint8_t icntl1)
{
    ASSERT_ERR((((uint32_t)icntl1 << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)icntl1 << 4));
}

__INLINE uint8_t iqgen_q_cntl0_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000000C)) >> 2);
}

__INLINE void iqgen_q_cntl0_setf(uint8_t qcntl0)
{
    ASSERT_ERR((((uint32_t)qcntl0 << 2) & ~((uint32_t)0x0000000C)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x0000000C)) | ((uint32_t)qcntl0 << 2));
}

__INLINE uint8_t iqgen_i_cntl0_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void iqgen_i_cntl0_setf(uint8_t icntl0)
{
    ASSERT_ERR((((uint32_t)icntl0 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_PL_WR(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR, (REG_PL_RD(IQGEN_IQ_SAMPLE_BEH_CNTL_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)icntl0 << 0));
}

/**
 * @brief I_SAMPLE_INIT_CNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            I_OFFSET3   0x0
 *  23:16            I_OFFSET2   0x0
 *  15:08            I_OFFSET1   0x0
 *  07:00            I_OFFSET0   0x0
 * </pre>
 */
#define IQGEN_I_SAMPLE_INIT_CNTL0_ADDR   0x1000A008
#define IQGEN_I_SAMPLE_INIT_CNTL0_OFFSET 0x00000008
#define IQGEN_I_SAMPLE_INIT_CNTL0_INDEX  0x00000002
#define IQGEN_I_SAMPLE_INIT_CNTL0_RESET  0x00000000

__INLINE uint32_t iqgen_i_sample_init_cntl0_get(void)
{
    return REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR);
}

__INLINE void iqgen_i_sample_init_cntl0_set(uint32_t value)
{
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR, value);
}

// field definitions
#define IQGEN_I_OFFSET3_MASK   ((uint32_t)0xFF000000)
#define IQGEN_I_OFFSET3_LSB    24
#define IQGEN_I_OFFSET3_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_I_OFFSET2_MASK   ((uint32_t)0x00FF0000)
#define IQGEN_I_OFFSET2_LSB    16
#define IQGEN_I_OFFSET2_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_I_OFFSET1_MASK   ((uint32_t)0x0000FF00)
#define IQGEN_I_OFFSET1_LSB    8
#define IQGEN_I_OFFSET1_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_I_OFFSET0_MASK   ((uint32_t)0x000000FF)
#define IQGEN_I_OFFSET0_LSB    0
#define IQGEN_I_OFFSET0_WIDTH  ((uint32_t)0x00000008)

#define IQGEN_I_OFFSET3_RST    0x0
#define IQGEN_I_OFFSET2_RST    0x0
#define IQGEN_I_OFFSET1_RST    0x0
#define IQGEN_I_OFFSET0_RST    0x0

__INLINE void iqgen_i_sample_init_cntl0_pack(uint8_t ioffset3, uint8_t ioffset2, uint8_t ioffset1, uint8_t ioffset0)
{
    ASSERT_ERR((((uint32_t)ioffset3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)ioffset2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)ioffset1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)ioffset0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR, ((uint32_t)ioffset3 << 24) | ((uint32_t)ioffset2 << 16) | ((uint32_t)ioffset1 << 8) | ((uint32_t)ioffset0 << 0));
}

__INLINE void iqgen_i_sample_init_cntl0_unpack(uint8_t *ioffset3, uint8_t *ioffset2, uint8_t *ioffset1, uint8_t *ioffset0)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR);

    *ioffset3 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *ioffset2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *ioffset1 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *ioffset0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t iqgen_i_offset3_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void iqgen_i_offset3_setf(uint8_t ioffset3)
{
    ASSERT_ERR((((uint32_t)ioffset3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)ioffset3 << 24));
}

__INLINE uint8_t iqgen_i_offset2_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void iqgen_i_offset2_setf(uint8_t ioffset2)
{
    ASSERT_ERR((((uint32_t)ioffset2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)ioffset2 << 16));
}

__INLINE uint8_t iqgen_i_offset1_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void iqgen_i_offset1_setf(uint8_t ioffset1)
{
    ASSERT_ERR((((uint32_t)ioffset1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)ioffset1 << 8));
}

__INLINE uint8_t iqgen_i_offset0_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void iqgen_i_offset0_setf(uint8_t ioffset0)
{
    ASSERT_ERR((((uint32_t)ioffset0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)ioffset0 << 0));
}

/**
 * @brief I_SAMPLE_INIT_CNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            I_OFFSET7   0x0
 *  23:16            I_OFFSET6   0x0
 *  15:08            I_OFFSET5   0x0
 *  07:00            I_OFFSET4   0x0
 * </pre>
 */
#define IQGEN_I_SAMPLE_INIT_CNTL1_ADDR   0x1000A00C
#define IQGEN_I_SAMPLE_INIT_CNTL1_OFFSET 0x0000000C
#define IQGEN_I_SAMPLE_INIT_CNTL1_INDEX  0x00000003
#define IQGEN_I_SAMPLE_INIT_CNTL1_RESET  0x00000000

__INLINE uint32_t iqgen_i_sample_init_cntl1_get(void)
{
    return REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR);
}

__INLINE void iqgen_i_sample_init_cntl1_set(uint32_t value)
{
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR, value);
}

// field definitions
#define IQGEN_I_OFFSET7_MASK   ((uint32_t)0xFF000000)
#define IQGEN_I_OFFSET7_LSB    24
#define IQGEN_I_OFFSET7_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_I_OFFSET6_MASK   ((uint32_t)0x00FF0000)
#define IQGEN_I_OFFSET6_LSB    16
#define IQGEN_I_OFFSET6_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_I_OFFSET5_MASK   ((uint32_t)0x0000FF00)
#define IQGEN_I_OFFSET5_LSB    8
#define IQGEN_I_OFFSET5_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_I_OFFSET4_MASK   ((uint32_t)0x000000FF)
#define IQGEN_I_OFFSET4_LSB    0
#define IQGEN_I_OFFSET4_WIDTH  ((uint32_t)0x00000008)

#define IQGEN_I_OFFSET7_RST    0x0
#define IQGEN_I_OFFSET6_RST    0x0
#define IQGEN_I_OFFSET5_RST    0x0
#define IQGEN_I_OFFSET4_RST    0x0

__INLINE void iqgen_i_sample_init_cntl1_pack(uint8_t ioffset7, uint8_t ioffset6, uint8_t ioffset5, uint8_t ioffset4)
{
    ASSERT_ERR((((uint32_t)ioffset7 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)ioffset6 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)ioffset5 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)ioffset4 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR, ((uint32_t)ioffset7 << 24) | ((uint32_t)ioffset6 << 16) | ((uint32_t)ioffset5 << 8) | ((uint32_t)ioffset4 << 0));
}

__INLINE void iqgen_i_sample_init_cntl1_unpack(uint8_t *ioffset7, uint8_t *ioffset6, uint8_t *ioffset5, uint8_t *ioffset4)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR);

    *ioffset7 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *ioffset6 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *ioffset5 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *ioffset4 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t iqgen_i_offset7_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void iqgen_i_offset7_setf(uint8_t ioffset7)
{
    ASSERT_ERR((((uint32_t)ioffset7 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)ioffset7 << 24));
}

__INLINE uint8_t iqgen_i_offset6_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void iqgen_i_offset6_setf(uint8_t ioffset6)
{
    ASSERT_ERR((((uint32_t)ioffset6 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)ioffset6 << 16));
}

__INLINE uint8_t iqgen_i_offset5_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void iqgen_i_offset5_setf(uint8_t ioffset5)
{
    ASSERT_ERR((((uint32_t)ioffset5 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)ioffset5 << 8));
}

__INLINE uint8_t iqgen_i_offset4_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void iqgen_i_offset4_setf(uint8_t ioffset4)
{
    ASSERT_ERR((((uint32_t)ioffset4 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_I_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)ioffset4 << 0));
}

/**
 * @brief Q_SAMPLE_INIT_CNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            Q_OFFSET3   0x0
 *  23:16            Q_OFFSET2   0x0
 *  15:08            Q_OFFSET1   0x0
 *  07:00            Q_OFFSET0   0x0
 * </pre>
 */
#define IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR   0x1000A010
#define IQGEN_Q_SAMPLE_INIT_CNTL0_OFFSET 0x00000010
#define IQGEN_Q_SAMPLE_INIT_CNTL0_INDEX  0x00000004
#define IQGEN_Q_SAMPLE_INIT_CNTL0_RESET  0x00000000

__INLINE uint32_t iqgen_q_sample_init_cntl0_get(void)
{
    return REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR);
}

__INLINE void iqgen_q_sample_init_cntl0_set(uint32_t value)
{
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR, value);
}

// field definitions
#define IQGEN_Q_OFFSET3_MASK   ((uint32_t)0xFF000000)
#define IQGEN_Q_OFFSET3_LSB    24
#define IQGEN_Q_OFFSET3_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_Q_OFFSET2_MASK   ((uint32_t)0x00FF0000)
#define IQGEN_Q_OFFSET2_LSB    16
#define IQGEN_Q_OFFSET2_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_Q_OFFSET1_MASK   ((uint32_t)0x0000FF00)
#define IQGEN_Q_OFFSET1_LSB    8
#define IQGEN_Q_OFFSET1_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_Q_OFFSET0_MASK   ((uint32_t)0x000000FF)
#define IQGEN_Q_OFFSET0_LSB    0
#define IQGEN_Q_OFFSET0_WIDTH  ((uint32_t)0x00000008)

#define IQGEN_Q_OFFSET3_RST    0x0
#define IQGEN_Q_OFFSET2_RST    0x0
#define IQGEN_Q_OFFSET1_RST    0x0
#define IQGEN_Q_OFFSET0_RST    0x0

__INLINE void iqgen_q_sample_init_cntl0_pack(uint8_t qoffset3, uint8_t qoffset2, uint8_t qoffset1, uint8_t qoffset0)
{
    ASSERT_ERR((((uint32_t)qoffset3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)qoffset2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)qoffset1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)qoffset0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR, ((uint32_t)qoffset3 << 24) | ((uint32_t)qoffset2 << 16) | ((uint32_t)qoffset1 << 8) | ((uint32_t)qoffset0 << 0));
}

__INLINE void iqgen_q_sample_init_cntl0_unpack(uint8_t *qoffset3, uint8_t *qoffset2, uint8_t *qoffset1, uint8_t *qoffset0)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR);

    *qoffset3 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *qoffset2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *qoffset1 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *qoffset0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t iqgen_q_offset3_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void iqgen_q_offset3_setf(uint8_t qoffset3)
{
    ASSERT_ERR((((uint32_t)qoffset3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)qoffset3 << 24));
}

__INLINE uint8_t iqgen_q_offset2_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void iqgen_q_offset2_setf(uint8_t qoffset2)
{
    ASSERT_ERR((((uint32_t)qoffset2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)qoffset2 << 16));
}

__INLINE uint8_t iqgen_q_offset1_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void iqgen_q_offset1_setf(uint8_t qoffset1)
{
    ASSERT_ERR((((uint32_t)qoffset1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)qoffset1 << 8));
}

__INLINE uint8_t iqgen_q_offset0_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void iqgen_q_offset0_setf(uint8_t qoffset0)
{
    ASSERT_ERR((((uint32_t)qoffset0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL0_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)qoffset0 << 0));
}

/**
 * @brief Q_SAMPLE_INIT_CNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            Q_OFFSET7   0x0
 *  23:16            Q_OFFSET6   0x0
 *  15:08            Q_OFFSET5   0x0
 *  07:00            Q_OFFSET4   0x0
 * </pre>
 */
#define IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR   0x1000A014
#define IQGEN_Q_SAMPLE_INIT_CNTL1_OFFSET 0x00000014
#define IQGEN_Q_SAMPLE_INIT_CNTL1_INDEX  0x00000005
#define IQGEN_Q_SAMPLE_INIT_CNTL1_RESET  0x00000000

__INLINE uint32_t iqgen_q_sample_init_cntl1_get(void)
{
    return REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR);
}

__INLINE void iqgen_q_sample_init_cntl1_set(uint32_t value)
{
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR, value);
}

// field definitions
#define IQGEN_Q_OFFSET7_MASK   ((uint32_t)0xFF000000)
#define IQGEN_Q_OFFSET7_LSB    24
#define IQGEN_Q_OFFSET7_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_Q_OFFSET6_MASK   ((uint32_t)0x00FF0000)
#define IQGEN_Q_OFFSET6_LSB    16
#define IQGEN_Q_OFFSET6_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_Q_OFFSET5_MASK   ((uint32_t)0x0000FF00)
#define IQGEN_Q_OFFSET5_LSB    8
#define IQGEN_Q_OFFSET5_WIDTH  ((uint32_t)0x00000008)
#define IQGEN_Q_OFFSET4_MASK   ((uint32_t)0x000000FF)
#define IQGEN_Q_OFFSET4_LSB    0
#define IQGEN_Q_OFFSET4_WIDTH  ((uint32_t)0x00000008)

#define IQGEN_Q_OFFSET7_RST    0x0
#define IQGEN_Q_OFFSET6_RST    0x0
#define IQGEN_Q_OFFSET5_RST    0x0
#define IQGEN_Q_OFFSET4_RST    0x0

__INLINE void iqgen_q_sample_init_cntl1_pack(uint8_t qoffset7, uint8_t qoffset6, uint8_t qoffset5, uint8_t qoffset4)
{
    ASSERT_ERR((((uint32_t)qoffset7 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)qoffset6 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)qoffset5 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)qoffset4 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR, ((uint32_t)qoffset7 << 24) | ((uint32_t)qoffset6 << 16) | ((uint32_t)qoffset5 << 8) | ((uint32_t)qoffset4 << 0));
}

__INLINE void iqgen_q_sample_init_cntl1_unpack(uint8_t *qoffset7, uint8_t *qoffset6, uint8_t *qoffset5, uint8_t *qoffset4)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR);

    *qoffset7 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *qoffset6 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *qoffset5 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *qoffset4 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t iqgen_q_offset7_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void iqgen_q_offset7_setf(uint8_t qoffset7)
{
    ASSERT_ERR((((uint32_t)qoffset7 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)qoffset7 << 24));
}

__INLINE uint8_t iqgen_q_offset6_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void iqgen_q_offset6_setf(uint8_t qoffset6)
{
    ASSERT_ERR((((uint32_t)qoffset6 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)qoffset6 << 16));
}

__INLINE uint8_t iqgen_q_offset5_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void iqgen_q_offset5_setf(uint8_t qoffset5)
{
    ASSERT_ERR((((uint32_t)qoffset5 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)qoffset5 << 8));
}

__INLINE uint8_t iqgen_q_offset4_getf(void)
{
    uint32_t localVal = REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void iqgen_q_offset4_setf(uint8_t qoffset4)
{
    ASSERT_ERR((((uint32_t)qoffset4 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR, (REG_PL_RD(IQGEN_Q_SAMPLE_INIT_CNTL1_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)qoffset4 << 0));
}


#endif // _REG_IQGEN_H_

