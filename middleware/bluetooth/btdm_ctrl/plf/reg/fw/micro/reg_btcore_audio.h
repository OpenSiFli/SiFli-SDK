#ifndef _REG_BTCORE_AUDIO_H_
#define _REG_BTCORE_AUDIO_H_

#include <stdint.h>
#include "_reg_btcore_audio.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_BTCORE_AUDIO_COUNT 157

#define REG_BTCORE_AUDIO_DECODING_MASK 0x000003FF

/**
 * @brief AUDIOCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:20        LINEAR_FORMAT   0x0
 *  17:16          SAMPLE_TYPE   0x0
 *     15              AULAWEN   0
 *  11:08           AULAW_CODE   0x0
 *     07               CVSDEN   0
 *     00        CVSD_BITORDER   0
 * </pre>
 */
#define BT_AUDIOCNTL_ADDR   0x40090670
#define BT_AUDIOCNTL_OFFSET 0x00000270
#define BT_AUDIOCNTL_INDEX  0x0000009C
#define BT_AUDIOCNTL_RESET  0x00000000

__INLINE uint32_t bt_audiocntl_get(int elt_idx)
{
    return REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);
}

__INLINE void bt_audiocntl_set(int elt_idx, uint32_t value)
{
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, value);
}

// field definitions
#define BT_LINEAR_FORMAT_MASK   ((uint32_t)0x00300000)
#define BT_LINEAR_FORMAT_LSB    20
#define BT_LINEAR_FORMAT_WIDTH  ((uint32_t)0x00000002)
#define BT_SAMPLE_TYPE_MASK     ((uint32_t)0x00030000)
#define BT_SAMPLE_TYPE_LSB      16
#define BT_SAMPLE_TYPE_WIDTH    ((uint32_t)0x00000002)
#define BT_AULAWEN_BIT          ((uint32_t)0x00008000)
#define BT_AULAWEN_POS          15
#define BT_AULAW_CODE_MASK      ((uint32_t)0x00000F00)
#define BT_AULAW_CODE_LSB       8
#define BT_AULAW_CODE_WIDTH     ((uint32_t)0x00000004)
#define BT_CVSDEN_BIT           ((uint32_t)0x00000080)
#define BT_CVSDEN_POS           7
#define BT_CVSD_BITORDER_BIT    ((uint32_t)0x00000001)
#define BT_CVSD_BITORDER_POS    0

#define BT_LINEAR_FORMAT_RST    0x0
#define BT_SAMPLE_TYPE_RST      0x0
#define BT_AULAWEN_RST          0x0
#define BT_AULAW_CODE_RST       0x0
#define BT_CVSDEN_RST           0x0
#define BT_CVSD_BITORDER_RST    0x0

__INLINE void bt_audiocntl_pack(int elt_idx, uint8_t linearformat, uint8_t sampletype, uint8_t aulawen, uint8_t aulawcode, uint8_t cvsden, uint8_t cvsdbitorder)
{
    ASSERT_ERR((((uint32_t)linearformat << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)sampletype << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)aulawen << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)aulawcode << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)cvsden << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)cvsdbitorder << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, ((uint32_t)linearformat << 20) | ((uint32_t)sampletype << 16) | ((uint32_t)aulawen << 15) | ((uint32_t)aulawcode << 8) | ((uint32_t)cvsden << 7) | ((uint32_t)cvsdbitorder << 0));
}

__INLINE void bt_audiocntl_unpack(int elt_idx, uint8_t *linearformat, uint8_t *sampletype, uint8_t *aulawen, uint8_t *aulawcode, uint8_t *cvsden, uint8_t *cvsdbitorder)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);

    *linearformat = (localVal & ((uint32_t)0x00300000)) >> 20;
    *sampletype = (localVal & ((uint32_t)0x00030000)) >> 16;
    *aulawen = (localVal & ((uint32_t)0x00008000)) >> 15;
    *aulawcode = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *cvsden = (localVal & ((uint32_t)0x00000080)) >> 7;
    *cvsdbitorder = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_audiocntl_linear_format_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void bt_audiocntl_linear_format_setf(int elt_idx, uint8_t linearformat)
{
    ASSERT_ERR((((uint32_t)linearformat << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, (REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE) & ~((uint32_t)0x00300000)) | ((uint32_t)linearformat << 20));
}

__INLINE uint8_t bt_audiocntl_sample_type_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_audiocntl_sample_type_setf(int elt_idx, uint8_t sampletype)
{
    ASSERT_ERR((((uint32_t)sampletype << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, (REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE) & ~((uint32_t)0x00030000)) | ((uint32_t)sampletype << 16));
}

__INLINE uint8_t bt_audiocntl_aulawen_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_audiocntl_aulawen_setf(int elt_idx, uint8_t aulawen)
{
    ASSERT_ERR((((uint32_t)aulawen << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, (REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE) & ~((uint32_t)0x00008000)) | ((uint32_t)aulawen << 15));
}

__INLINE uint8_t bt_audiocntl_aulaw_code_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_audiocntl_aulaw_code_setf(int elt_idx, uint8_t aulawcode)
{
    ASSERT_ERR((((uint32_t)aulawcode << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, (REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE) & ~((uint32_t)0x00000F00)) | ((uint32_t)aulawcode << 8));
}

__INLINE uint8_t bt_audiocntl_cvsden_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_audiocntl_cvsden_setf(int elt_idx, uint8_t cvsden)
{
    ASSERT_ERR((((uint32_t)cvsden << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, (REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE) & ~((uint32_t)0x00000080)) | ((uint32_t)cvsden << 7));
}

__INLINE uint8_t bt_audiocntl_cvsd_bitorder_getf(int elt_idx)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_audiocntl_cvsd_bitorder_setf(int elt_idx, uint8_t cvsdbitorder)
{
    ASSERT_ERR((((uint32_t)cvsdbitorder << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE, (REG_BT_RD(BT_AUDIOCNTL_ADDR + elt_idx * REG_BTCORE_AUDIO_SIZE) & ~((uint32_t)0x00000001)) | ((uint32_t)cvsdbitorder << 0));
}


#endif // _REG_BTCORE_AUDIO_H_

