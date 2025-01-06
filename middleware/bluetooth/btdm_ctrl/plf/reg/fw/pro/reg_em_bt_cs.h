#ifndef _REG_EM_BT_CS_H_
#define _REG_EM_BT_CS_H_

#include <stdint.h>
#include "_reg_em_bt_cs.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BT_CS_COUNT 48

#define REG_EM_BT_CS_DECODING_MASK 0x0000007F

#define REG_EM_BT_CS_ADDR_GET(idx) (EM_BT_CS_OFFSET + (idx) * REG_EM_BT_CS_SIZE)

/**
 * @brief FRCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15                FPOLL   0
 *  14:12             FWMSPATT   0x0
 *     10             TXBSY_EN   0
 *     09             RXBSY_EN   0
 *     08              DNABORT   0
 *     07               SAM_EN   0
 *     06             LSAM_DSB   0
 *  04:00               FORMAT   0x0
 * </pre>
 */
#define EM_BT_FRCNTL_ADDR   (0x204F0000 + EM_BT_CS_OFFSET)
#define EM_BT_FRCNTL_INDEX  0x00000000
#define EM_BT_FRCNTL_RESET  0x00000000

__INLINE uint16_t em_bt_frcntl_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_frcntl_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_FPOLL_BIT       ((uint16_t)0x00008000)
#define EM_BT_FPOLL_POS       15
#define EM_BT_FWMSPATT_MASK   ((uint16_t)0x00007000)
#define EM_BT_FWMSPATT_LSB    12
#define EM_BT_FWMSPATT_WIDTH  ((uint16_t)0x00000003)
#define EM_BT_TXBSY_EN_BIT    ((uint16_t)0x00000400)
#define EM_BT_TXBSY_EN_POS    10
#define EM_BT_RXBSY_EN_BIT    ((uint16_t)0x00000200)
#define EM_BT_RXBSY_EN_POS    9
#define EM_BT_DNABORT_BIT     ((uint16_t)0x00000100)
#define EM_BT_DNABORT_POS     8
#define EM_BT_SAM_EN_BIT      ((uint16_t)0x00000080)
#define EM_BT_SAM_EN_POS      7
#define EM_BT_LSAM_DSB_BIT    ((uint16_t)0x00000040)
#define EM_BT_LSAM_DSB_POS    6
#define EM_BT_FORMAT_MASK     ((uint16_t)0x0000001F)
#define EM_BT_FORMAT_LSB      0
#define EM_BT_FORMAT_WIDTH    ((uint16_t)0x00000005)

#define EM_BT_FPOLL_RST       0x0
#define EM_BT_FWMSPATT_RST    0x0
#define EM_BT_TXBSY_EN_RST    0x0
#define EM_BT_RXBSY_EN_RST    0x0
#define EM_BT_DNABORT_RST     0x0
#define EM_BT_SAM_EN_RST      0x0
#define EM_BT_LSAM_DSB_RST    0x0
#define EM_BT_FORMAT_RST      0x0

__INLINE void em_bt_frcntl_pack(int elt_idx, uint8_t fpoll, uint8_t fwmspatt, uint8_t txbsyen, uint8_t rxbsyen, uint8_t dnabort, uint8_t samen, uint8_t lsamdsb, uint8_t format)
{
    ASSERT_ERR((((uint16_t)fpoll << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)fwmspatt << 12) & ~((uint16_t)0x00007000)) == 0);
    ASSERT_ERR((((uint16_t)txbsyen << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)rxbsyen << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)dnabort << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)samen << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)lsamdsb << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)format << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)fpoll << 15) | ((uint16_t)fwmspatt << 12) | ((uint16_t)txbsyen << 10) | ((uint16_t)rxbsyen << 9) | ((uint16_t)dnabort << 8) | ((uint16_t)samen << 7) | ((uint16_t)lsamdsb << 6) | ((uint16_t)format << 0));
}

__INLINE void em_bt_frcntl_unpack(int elt_idx, uint8_t *fpoll, uint8_t *fwmspatt, uint8_t *txbsyen, uint8_t *rxbsyen, uint8_t *dnabort, uint8_t *samen, uint8_t *lsamdsb, uint8_t *format)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *fpoll = (localVal & ((uint16_t)0x00008000)) >> 15;
    *fwmspatt = (localVal & ((uint16_t)0x00007000)) >> 12;
    *txbsyen = (localVal & ((uint16_t)0x00000400)) >> 10;
    *rxbsyen = (localVal & ((uint16_t)0x00000200)) >> 9;
    *dnabort = (localVal & ((uint16_t)0x00000100)) >> 8;
    *samen = (localVal & ((uint16_t)0x00000080)) >> 7;
    *lsamdsb = (localVal & ((uint16_t)0x00000040)) >> 6;
    *format = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_bt_frcntl_fpoll_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_frcntl_fpoll_setf(int elt_idx, uint8_t fpoll)
{
    ASSERT_ERR((((uint16_t)fpoll << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)fpoll << 15));
}

__INLINE uint8_t em_bt_frcntl_fwmspatt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00007000)) >> 12);
}

__INLINE void em_bt_frcntl_fwmspatt_setf(int elt_idx, uint8_t fwmspatt)
{
    ASSERT_ERR((((uint16_t)fwmspatt << 12) & ~((uint16_t)0x00007000)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00007000)) | ((uint16_t)fwmspatt << 12));
}

__INLINE uint8_t em_bt_frcntl_txbsy_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_bt_frcntl_txbsy_en_setf(int elt_idx, uint8_t txbsyen)
{
    ASSERT_ERR((((uint16_t)txbsyen << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)txbsyen << 10));
}

__INLINE uint8_t em_bt_frcntl_rxbsy_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_bt_frcntl_rxbsy_en_setf(int elt_idx, uint8_t rxbsyen)
{
    ASSERT_ERR((((uint16_t)rxbsyen << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)rxbsyen << 9));
}

__INLINE uint8_t em_bt_frcntl_dnabort_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_bt_frcntl_dnabort_setf(int elt_idx, uint8_t dnabort)
{
    ASSERT_ERR((((uint16_t)dnabort << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)dnabort << 8));
}

__INLINE uint8_t em_bt_frcntl_sam_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_bt_frcntl_sam_en_setf(int elt_idx, uint8_t samen)
{
    ASSERT_ERR((((uint16_t)samen << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)samen << 7));
}

__INLINE uint8_t em_bt_frcntl_lsam_dsb_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_bt_frcntl_lsam_dsb_setf(int elt_idx, uint8_t lsamdsb)
{
    ASSERT_ERR((((uint16_t)lsamdsb << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)lsamdsb << 6));
}

__INLINE uint8_t em_bt_frcntl_format_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_bt_frcntl_format_setf(int elt_idx, uint8_t format)
{
    ASSERT_ERR((((uint16_t)format << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_FRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)format << 0));
}

/**
 * @brief CLKOFF0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              CLKOFF0   0x0
 * </pre>
 */
#define EM_BT_CLKOFF0_ADDR   (0x204F0002 + EM_BT_CS_OFFSET)
#define EM_BT_CLKOFF0_INDEX  0x00000001
#define EM_BT_CLKOFF0_RESET  0x00000000

__INLINE uint16_t em_bt_clkoff0_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_CLKOFF0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_clkoff0_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_CLKOFF0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_CLKOFF0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_CLKOFF0_LSB    0
#define EM_BT_CLKOFF0_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_CLKOFF0_RST    0x0

__INLINE uint16_t em_bt_clkoff0_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_CLKOFF0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_clkoff0_setf(int elt_idx, uint16_t clkoff0)
{
    ASSERT_ERR((((uint16_t)clkoff0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_CLKOFF0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)clkoff0 << 0);
}

/**
 * @brief CLKOFF1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:00              CLKOFF1   0x0
 * </pre>
 */
#define EM_BT_CLKOFF1_ADDR   (0x204F0004 + EM_BT_CS_OFFSET)
#define EM_BT_CLKOFF1_INDEX  0x00000002
#define EM_BT_CLKOFF1_RESET  0x00000000

__INLINE uint16_t em_bt_clkoff1_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_CLKOFF1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_clkoff1_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_CLKOFF1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_CLKOFF1_MASK   ((uint16_t)0x00000FFF)
#define EM_BT_CLKOFF1_LSB    0
#define EM_BT_CLKOFF1_WIDTH  ((uint16_t)0x0000000C)

#define EM_BT_CLKOFF1_RST    0x0

__INLINE uint16_t em_bt_clkoff1_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_CLKOFF1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00000FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_clkoff1_setf(int elt_idx, uint16_t clkoff1)
{
    ASSERT_ERR((((uint16_t)clkoff1 << 0) & ~((uint16_t)0x00000FFF)) == 0);
    EM_BT_WR(EM_BT_CLKOFF1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)clkoff1 << 0);
}

/**
 * @brief LINKCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               AKNENA   0
 *     14               AFHENA   0
 *     13                 LAAP   0
 *     12                WHDSB   0
 *     11               ACLEDR   0
 *  10:08            ACLLTADDR   0x0
 *  04:00              LINKLBL   0x0
 * </pre>
 */
#define EM_BT_LINKCNTL_ADDR   (0x204F0006 + EM_BT_CS_OFFSET)
#define EM_BT_LINKCNTL_INDEX  0x00000003
#define EM_BT_LINKCNTL_RESET  0x00000000

__INLINE uint16_t em_bt_linkcntl_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_linkcntl_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_AKNENA_BIT       ((uint16_t)0x00008000)
#define EM_BT_AKNENA_POS       15
#define EM_BT_AFHENA_BIT       ((uint16_t)0x00004000)
#define EM_BT_AFHENA_POS       14
#define EM_BT_LAAP_BIT         ((uint16_t)0x00002000)
#define EM_BT_LAAP_POS         13
#define EM_BT_WHDSB_BIT        ((uint16_t)0x00001000)
#define EM_BT_WHDSB_POS        12
#define EM_BT_ACLEDR_BIT       ((uint16_t)0x00000800)
#define EM_BT_ACLEDR_POS       11
#define EM_BT_ACLLTADDR_MASK   ((uint16_t)0x00000700)
#define EM_BT_ACLLTADDR_LSB    8
#define EM_BT_ACLLTADDR_WIDTH  ((uint16_t)0x00000003)
#define EM_BT_LINKLBL_MASK     ((uint16_t)0x0000001F)
#define EM_BT_LINKLBL_LSB      0
#define EM_BT_LINKLBL_WIDTH    ((uint16_t)0x00000005)

#define EM_BT_AKNENA_RST       0x0
#define EM_BT_AFHENA_RST       0x0
#define EM_BT_LAAP_RST         0x0
#define EM_BT_WHDSB_RST        0x0
#define EM_BT_ACLEDR_RST       0x0
#define EM_BT_ACLLTADDR_RST    0x0
#define EM_BT_LINKLBL_RST      0x0

__INLINE void em_bt_linkcntl_pack(int elt_idx, uint8_t aknena, uint8_t afhena, uint8_t laap, uint8_t whdsb, uint8_t acledr, uint8_t aclltaddr, uint8_t linklbl)
{
    ASSERT_ERR((((uint16_t)aknena << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)afhena << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)laap << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)whdsb << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)acledr << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)aclltaddr << 8) & ~((uint16_t)0x00000700)) == 0);
    ASSERT_ERR((((uint16_t)linklbl << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)aknena << 15) | ((uint16_t)afhena << 14) | ((uint16_t)laap << 13) | ((uint16_t)whdsb << 12) | ((uint16_t)acledr << 11) | ((uint16_t)aclltaddr << 8) | ((uint16_t)linklbl << 0));
}

__INLINE void em_bt_linkcntl_unpack(int elt_idx, uint8_t *aknena, uint8_t *afhena, uint8_t *laap, uint8_t *whdsb, uint8_t *acledr, uint8_t *aclltaddr, uint8_t *linklbl)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *aknena = (localVal & ((uint16_t)0x00008000)) >> 15;
    *afhena = (localVal & ((uint16_t)0x00004000)) >> 14;
    *laap = (localVal & ((uint16_t)0x00002000)) >> 13;
    *whdsb = (localVal & ((uint16_t)0x00001000)) >> 12;
    *acledr = (localVal & ((uint16_t)0x00000800)) >> 11;
    *aclltaddr = (localVal & ((uint16_t)0x00000700)) >> 8;
    *linklbl = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_bt_linkcntl_aknena_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_linkcntl_aknena_setf(int elt_idx, uint8_t aknena)
{
    ASSERT_ERR((((uint16_t)aknena << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)aknena << 15));
}

__INLINE uint8_t em_bt_linkcntl_afhena_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_bt_linkcntl_afhena_setf(int elt_idx, uint8_t afhena)
{
    ASSERT_ERR((((uint16_t)afhena << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)afhena << 14));
}

__INLINE uint8_t em_bt_linkcntl_laap_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_bt_linkcntl_laap_setf(int elt_idx, uint8_t laap)
{
    ASSERT_ERR((((uint16_t)laap << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)laap << 13));
}

__INLINE uint8_t em_bt_linkcntl_whdsb_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_bt_linkcntl_whdsb_setf(int elt_idx, uint8_t whdsb)
{
    ASSERT_ERR((((uint16_t)whdsb << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)whdsb << 12));
}

__INLINE uint8_t em_bt_linkcntl_acledr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_bt_linkcntl_acledr_setf(int elt_idx, uint8_t acledr)
{
    ASSERT_ERR((((uint16_t)acledr << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)acledr << 11));
}

__INLINE uint8_t em_bt_linkcntl_aclltaddr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000700)) >> 8);
}

__INLINE void em_bt_linkcntl_aclltaddr_setf(int elt_idx, uint8_t aclltaddr)
{
    ASSERT_ERR((((uint16_t)aclltaddr << 8) & ~((uint16_t)0x00000700)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000700)) | ((uint16_t)aclltaddr << 8));
}

__INLINE uint8_t em_bt_linkcntl_linklbl_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_bt_linkcntl_linklbl_setf(int elt_idx, uint8_t linklbl)
{
    ASSERT_ERR((((uint16_t)linklbl << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LINKCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)linklbl << 0));
}

/**
 * @brief BDADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               BDADDR   0x0
 * </pre>
 */
#define EM_BT_BDADDR_ADDR   (0x204F0008 + EM_BT_CS_OFFSET)
#define EM_BT_BDADDR_INDEX  0x00000004
#define EM_BT_BDADDR_RESET  0x00000000
#define EM_BT_BDADDR_COUNT  3

__INLINE uint16_t em_bt_bdaddr_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BT_RD(EM_BT_BDADDR_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2);
}

__INLINE void em_bt_bdaddr_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BT_WR(EM_BT_BDADDR_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BT_BDADDR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_BDADDR_LSB    0
#define EM_BT_BDADDR_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_BDADDR_RST    0x0

__INLINE uint16_t em_bt_bdaddr_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BT_RD(EM_BT_BDADDR_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_bdaddr_setf(int elt_idx, int reg_idx, uint16_t bdaddr)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)bdaddr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_BDADDR_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2, (uint16_t)bdaddr << 0);
}

/**
 * @brief BCH0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                 BCH0   0x0
 * </pre>
 */
#define EM_BT_BCH0_ADDR   (0x204F000E + EM_BT_CS_OFFSET)
#define EM_BT_BCH0_INDEX  0x00000007
#define EM_BT_BCH0_RESET  0x00000000

__INLINE uint16_t em_bt_bch0_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_BCH0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_bch0_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_BCH0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_BCH0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_BCH0_LSB    0
#define EM_BT_BCH0_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_BCH0_RST    0x0

__INLINE uint16_t em_bt_bch0_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_BCH0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_bch0_setf(int elt_idx, uint16_t bch0)
{
    ASSERT_ERR((((uint16_t)bch0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_BCH0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)bch0 << 0);
}

/**
 * @brief BCH1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                 BCH1   0x0
 * </pre>
 */
#define EM_BT_BCH1_ADDR   (0x204F0010 + EM_BT_CS_OFFSET)
#define EM_BT_BCH1_INDEX  0x00000008
#define EM_BT_BCH1_RESET  0x00000000

__INLINE uint16_t em_bt_bch1_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_BCH1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_bch1_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_BCH1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_BCH1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_BCH1_LSB    0
#define EM_BT_BCH1_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_BCH1_RST    0x0

__INLINE uint16_t em_bt_bch1_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_BCH1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_bch1_setf(int elt_idx, uint16_t bch1)
{
    ASSERT_ERR((((uint16_t)bch1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_BCH1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)bch1 << 0);
}

/**
 * @brief RXMAXBUF_BCH2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:06             RXMAXBUF   0x0
 *  01:00                 BCH2   0x0
 * </pre>
 */
#define EM_BT_RXMAXBUF_BCH2_ADDR   (0x204F0012 + EM_BT_CS_OFFSET)
#define EM_BT_RXMAXBUF_BCH2_INDEX  0x00000009
#define EM_BT_RXMAXBUF_BCH2_RESET  0x00000000

__INLINE uint16_t em_bt_rxmaxbuf_bch2_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_rxmaxbuf_bch2_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_RXMAXBUF_MASK   ((uint16_t)0x0000FFC0)
#define EM_BT_RXMAXBUF_LSB    6
#define EM_BT_RXMAXBUF_WIDTH  ((uint16_t)0x0000000A)
#define EM_BT_BCH2_MASK       ((uint16_t)0x00000003)
#define EM_BT_BCH2_LSB        0
#define EM_BT_BCH2_WIDTH      ((uint16_t)0x00000002)

#define EM_BT_RXMAXBUF_RST    0x0
#define EM_BT_BCH2_RST        0x0

__INLINE void em_bt_rxmaxbuf_bch2_pack(int elt_idx, uint16_t rxmaxbuf, uint8_t bch2)
{
    ASSERT_ERR((((uint16_t)rxmaxbuf << 6) & ~((uint16_t)0x0000FFC0)) == 0);
    ASSERT_ERR((((uint16_t)bch2 << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BT_WR(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)rxmaxbuf << 6) | ((uint16_t)bch2 << 0));
}

__INLINE void em_bt_rxmaxbuf_bch2_unpack(int elt_idx, uint16_t *rxmaxbuf, uint8_t *bch2)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *rxmaxbuf = (localVal & ((uint16_t)0x0000FFC0)) >> 6;
    *bch2 = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint16_t em_bt_rxmaxbuf_bch2_rxmaxbuf_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FFC0)) >> 6);
}

__INLINE void em_bt_rxmaxbuf_bch2_rxmaxbuf_setf(int elt_idx, uint16_t rxmaxbuf)
{
    ASSERT_ERR((((uint16_t)rxmaxbuf << 6) & ~((uint16_t)0x0000FFC0)) == 0);
    EM_BT_WR(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000FFC0)) | ((uint16_t)rxmaxbuf << 6));
}

__INLINE uint8_t em_bt_rxmaxbuf_bch2_bch2_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_bt_rxmaxbuf_bch2_bch2_setf(int elt_idx, uint8_t bch2)
{
    ASSERT_ERR((((uint16_t)bch2 << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BT_WR(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_RXMAXBUF_BCH2_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)bch2 << 0));
}

/**
 * @brief PWRCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15                FH_EN   0
 *  14:08                 FREQ   0x0
 *  07:00                TXPWR   0x0
 * </pre>
 */
#define EM_BT_PWRCNTL_ADDR   (0x204F0014 + EM_BT_CS_OFFSET)
#define EM_BT_PWRCNTL_INDEX  0x0000000A
#define EM_BT_PWRCNTL_RESET  0x00000000

__INLINE uint16_t em_bt_pwrcntl_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_pwrcntl_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_FH_EN_BIT    ((uint16_t)0x00008000)
#define EM_BT_FH_EN_POS    15
#define EM_BT_FREQ_MASK    ((uint16_t)0x00007F00)
#define EM_BT_FREQ_LSB     8
#define EM_BT_FREQ_WIDTH   ((uint16_t)0x00000007)
#define EM_BT_TXPWR_MASK   ((uint16_t)0x000000FF)
#define EM_BT_TXPWR_LSB    0
#define EM_BT_TXPWR_WIDTH  ((uint16_t)0x00000008)

#define EM_BT_FH_EN_RST    0x0
#define EM_BT_FREQ_RST     0x0
#define EM_BT_TXPWR_RST    0x0

__INLINE void em_bt_pwrcntl_pack(int elt_idx, uint8_t fhen, uint8_t freq, uint8_t txpwr)
{
    ASSERT_ERR((((uint16_t)fhen << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)freq << 8) & ~((uint16_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint16_t)txpwr << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BT_WR(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)fhen << 15) | ((uint16_t)freq << 8) | ((uint16_t)txpwr << 0));
}

__INLINE void em_bt_pwrcntl_unpack(int elt_idx, uint8_t *fhen, uint8_t *freq, uint8_t *txpwr)
{
    uint16_t localVal = EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *fhen = (localVal & ((uint16_t)0x00008000)) >> 15;
    *freq = (localVal & ((uint16_t)0x00007F00)) >> 8;
    *txpwr = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_bt_pwrcntl_fh_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_pwrcntl_fh_en_setf(int elt_idx, uint8_t fhen)
{
    ASSERT_ERR((((uint16_t)fhen << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)fhen << 15));
}

__INLINE uint8_t em_bt_pwrcntl_freq_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00007F00)) >> 8);
}

__INLINE void em_bt_pwrcntl_freq_setf(int elt_idx, uint8_t freq)
{
    ASSERT_ERR((((uint16_t)freq << 8) & ~((uint16_t)0x00007F00)) == 0);
    EM_BT_WR(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00007F00)) | ((uint16_t)freq << 8));
}

__INLINE uint8_t em_bt_pwrcntl_txpwr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_bt_pwrcntl_txpwr_setf(int elt_idx, uint8_t txpwr)
{
    ASSERT_ERR((((uint16_t)txpwr << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BT_WR(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_PWRCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)txpwr << 0));
}

/**
 * @brief TXRXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:13                RXTHR   0x0
 *     11                RXEIR   0
 *     10               RXBCRY   0
 *  09:08              RXCRYPT   0x0
 *     07              STOPMOD   0
 *     06              NULLFLT   0
 *     04            EXT_PA_EN   0
 *     03                TXEIR   0
 *     02               TXBCRY   0
 *  01:00              TXCRYPT   0x0
 * </pre>
 */
#define EM_BT_TXRXCNTL_ADDR   (0x204F0016 + EM_BT_CS_OFFSET)
#define EM_BT_TXRXCNTL_INDEX  0x0000000B
#define EM_BT_TXRXCNTL_RESET  0x00000000

__INLINE uint16_t em_bt_txrxcntl_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_txrxcntl_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_RXTHR_MASK       ((uint16_t)0x0000E000)
#define EM_BT_RXTHR_LSB        13
#define EM_BT_RXTHR_WIDTH      ((uint16_t)0x00000003)
#define EM_BT_RXEIR_BIT        ((uint16_t)0x00000800)
#define EM_BT_RXEIR_POS        11
#define EM_BT_RXBCRY_BIT       ((uint16_t)0x00000400)
#define EM_BT_RXBCRY_POS       10
#define EM_BT_RXCRYPT_MASK     ((uint16_t)0x00000300)
#define EM_BT_RXCRYPT_LSB      8
#define EM_BT_RXCRYPT_WIDTH    ((uint16_t)0x00000002)
#define EM_BT_STOPMOD_BIT      ((uint16_t)0x00000080)
#define EM_BT_STOPMOD_POS      7
#define EM_BT_NULLFLT_BIT      ((uint16_t)0x00000040)
#define EM_BT_NULLFLT_POS      6
#define EM_BT_EXT_PA_EN_BIT    ((uint16_t)0x00000010)
#define EM_BT_EXT_PA_EN_POS    4
#define EM_BT_TXEIR_BIT        ((uint16_t)0x00000008)
#define EM_BT_TXEIR_POS        3
#define EM_BT_TXBCRY_BIT       ((uint16_t)0x00000004)
#define EM_BT_TXBCRY_POS       2
#define EM_BT_TXCRYPT_MASK     ((uint16_t)0x00000003)
#define EM_BT_TXCRYPT_LSB      0
#define EM_BT_TXCRYPT_WIDTH    ((uint16_t)0x00000002)

#define EM_BT_RXTHR_RST        0x0
#define EM_BT_RXEIR_RST        0x0
#define EM_BT_RXBCRY_RST       0x0
#define EM_BT_RXCRYPT_RST      0x0
#define EM_BT_STOPMOD_RST      0x0
#define EM_BT_NULLFLT_RST      0x0
#define EM_BT_EXT_PA_EN_RST    0x0
#define EM_BT_TXEIR_RST        0x0
#define EM_BT_TXBCRY_RST       0x0
#define EM_BT_TXCRYPT_RST      0x0

__INLINE void em_bt_txrxcntl_pack(int elt_idx, uint8_t rxthr, uint8_t rxeir, uint8_t rxbcry, uint8_t rxcrypt, uint8_t stopmod, uint8_t nullflt, uint8_t extpaen, uint8_t txeir, uint8_t txbcry, uint8_t txcrypt)
{
    ASSERT_ERR((((uint16_t)rxthr << 13) & ~((uint16_t)0x0000E000)) == 0);
    ASSERT_ERR((((uint16_t)rxeir << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)rxbcry << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)rxcrypt << 8) & ~((uint16_t)0x00000300)) == 0);
    ASSERT_ERR((((uint16_t)stopmod << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)nullflt << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)extpaen << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)txeir << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)txbcry << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)txcrypt << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)rxthr << 13) | ((uint16_t)rxeir << 11) | ((uint16_t)rxbcry << 10) | ((uint16_t)rxcrypt << 8) | ((uint16_t)stopmod << 7) | ((uint16_t)nullflt << 6) | ((uint16_t)extpaen << 4) | ((uint16_t)txeir << 3) | ((uint16_t)txbcry << 2) | ((uint16_t)txcrypt << 0));
}

__INLINE void em_bt_txrxcntl_unpack(int elt_idx, uint8_t *rxthr, uint8_t *rxeir, uint8_t *rxbcry, uint8_t *rxcrypt, uint8_t *stopmod, uint8_t *nullflt, uint8_t *extpaen, uint8_t *txeir, uint8_t *txbcry, uint8_t *txcrypt)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *rxthr = (localVal & ((uint16_t)0x0000E000)) >> 13;
    *rxeir = (localVal & ((uint16_t)0x00000800)) >> 11;
    *rxbcry = (localVal & ((uint16_t)0x00000400)) >> 10;
    *rxcrypt = (localVal & ((uint16_t)0x00000300)) >> 8;
    *stopmod = (localVal & ((uint16_t)0x00000080)) >> 7;
    *nullflt = (localVal & ((uint16_t)0x00000040)) >> 6;
    *extpaen = (localVal & ((uint16_t)0x00000010)) >> 4;
    *txeir = (localVal & ((uint16_t)0x00000008)) >> 3;
    *txbcry = (localVal & ((uint16_t)0x00000004)) >> 2;
    *txcrypt = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_bt_txrxcntl_rxthr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000E000)) >> 13);
}

__INLINE void em_bt_txrxcntl_rxthr_setf(int elt_idx, uint8_t rxthr)
{
    ASSERT_ERR((((uint16_t)rxthr << 13) & ~((uint16_t)0x0000E000)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000E000)) | ((uint16_t)rxthr << 13));
}

__INLINE uint8_t em_bt_txrxcntl_rxeir_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_bt_txrxcntl_rxeir_setf(int elt_idx, uint8_t rxeir)
{
    ASSERT_ERR((((uint16_t)rxeir << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)rxeir << 11));
}

__INLINE uint8_t em_bt_txrxcntl_rxbcry_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_bt_txrxcntl_rxbcry_setf(int elt_idx, uint8_t rxbcry)
{
    ASSERT_ERR((((uint16_t)rxbcry << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)rxbcry << 10));
}

__INLINE uint8_t em_bt_txrxcntl_rxcrypt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000300)) >> 8);
}

__INLINE void em_bt_txrxcntl_rxcrypt_setf(int elt_idx, uint8_t rxcrypt)
{
    ASSERT_ERR((((uint16_t)rxcrypt << 8) & ~((uint16_t)0x00000300)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000300)) | ((uint16_t)rxcrypt << 8));
}

__INLINE uint8_t em_bt_txrxcntl_stopmod_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_bt_txrxcntl_stopmod_setf(int elt_idx, uint8_t stopmod)
{
    ASSERT_ERR((((uint16_t)stopmod << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)stopmod << 7));
}

__INLINE uint8_t em_bt_txrxcntl_nullflt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_bt_txrxcntl_nullflt_setf(int elt_idx, uint8_t nullflt)
{
    ASSERT_ERR((((uint16_t)nullflt << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)nullflt << 6));
}

__INLINE uint8_t em_bt_txrxcntl_ext_pa_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_bt_txrxcntl_ext_pa_en_setf(int elt_idx, uint8_t extpaen)
{
    ASSERT_ERR((((uint16_t)extpaen << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)extpaen << 4));
}

__INLINE uint8_t em_bt_txrxcntl_txeir_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_bt_txrxcntl_txeir_setf(int elt_idx, uint8_t txeir)
{
    ASSERT_ERR((((uint16_t)txeir << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)txeir << 3));
}

__INLINE uint8_t em_bt_txrxcntl_txbcry_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_bt_txrxcntl_txbcry_setf(int elt_idx, uint8_t txbcry)
{
    ASSERT_ERR((((uint16_t)txbcry << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)txbcry << 2));
}

__INLINE uint8_t em_bt_txrxcntl_txcrypt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_bt_txrxcntl_txcrypt_setf(int elt_idx, uint8_t txcrypt)
{
    ASSERT_ERR((((uint16_t)txcrypt << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BT_WR(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXRXCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)txcrypt << 0));
}

/**
 * @brief WINCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               RXWIDE   0
 *  12:00              RXWINSZ   0x0
 * </pre>
 */
#define EM_BT_WINCNTL_ADDR   (0x204F0018 + EM_BT_CS_OFFSET)
#define EM_BT_WINCNTL_INDEX  0x0000000C
#define EM_BT_WINCNTL_RESET  0x00000000

__INLINE uint16_t em_bt_wincntl_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_wincntl_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_RXWIDE_BIT     ((uint16_t)0x00008000)
#define EM_BT_RXWIDE_POS     15
#define EM_BT_RXWINSZ_MASK   ((uint16_t)0x00001FFF)
#define EM_BT_RXWINSZ_LSB    0
#define EM_BT_RXWINSZ_WIDTH  ((uint16_t)0x0000000D)

#define EM_BT_RXWIDE_RST     0x0
#define EM_BT_RXWINSZ_RST    0x0

__INLINE void em_bt_wincntl_pack(int elt_idx, uint8_t rxwide, uint16_t rxwinsz)
{
    ASSERT_ERR((((uint16_t)rxwide << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)rxwinsz << 0) & ~((uint16_t)0x00001FFF)) == 0);
    EM_BT_WR(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)rxwide << 15) | ((uint16_t)rxwinsz << 0));
}

__INLINE void em_bt_wincntl_unpack(int elt_idx, uint8_t *rxwide, uint16_t *rxwinsz)
{
    uint16_t localVal = EM_BT_RD(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *rxwide = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxwinsz = (localVal & ((uint16_t)0x00001FFF)) >> 0;
}

__INLINE uint8_t em_bt_wincntl_rxwide_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_wincntl_rxwide_setf(int elt_idx, uint8_t rxwide)
{
    ASSERT_ERR((((uint16_t)rxwide << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxwide << 15));
}

__INLINE uint16_t em_bt_wincntl_rxwinsz_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001FFF)) >> 0);
}

__INLINE void em_bt_wincntl_rxwinsz_setf(int elt_idx, uint16_t rxwinsz)
{
    ASSERT_ERR((((uint16_t)rxwinsz << 0) & ~((uint16_t)0x00001FFF)) == 0);
    EM_BT_WR(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_WINCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00001FFF)) | ((uint16_t)rxwinsz << 0));
}

/**
 * @brief TXDESCPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00            TXDESCPTR   0x0
 * </pre>
 */
#define EM_BT_TXDESCPTR_ADDR   (0x204F001A + EM_BT_CS_OFFSET)
#define EM_BT_TXDESCPTR_INDEX  0x0000000D
#define EM_BT_TXDESCPTR_RESET  0x00000000

__INLINE uint16_t em_bt_txdescptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXDESCPTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_txdescptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXDESCPTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_TXDESCPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BT_TXDESCPTR_LSB    0
#define EM_BT_TXDESCPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BT_TXDESCPTR_RST    0x0

__INLINE uint16_t em_bt_txdescptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXDESCPTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_txdescptr_setf(int elt_idx, uint16_t txdescptr)
{
    ASSERT_ERR((((uint16_t)txdescptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BT_WR(EM_BT_TXDESCPTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)txdescptr << 0);
}

/**
 * @brief CHMAP0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               CHMAP0   0xFFFF
 * </pre>
 */
#define EM_BT_CHMAP0_ADDR   (0x204F001C + EM_BT_CS_OFFSET)
#define EM_BT_CHMAP0_INDEX  0x0000000E
#define EM_BT_CHMAP0_RESET  0x0000FFFF

__INLINE uint16_t em_bt_chmap0_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_CHMAP0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_chmap0_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_CHMAP0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_CHMAP0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_CHMAP0_LSB    0
#define EM_BT_CHMAP0_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_CHMAP0_RST    0xFFFF

__INLINE uint16_t em_bt_chmap0_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_CHMAP0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_chmap0_setf(int elt_idx, uint16_t chmap0)
{
    ASSERT_ERR((((uint16_t)chmap0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_CHMAP0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)chmap0 << 0);
}

/**
 * @brief CHMAP1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               CHMAP1   0xFFFF
 * </pre>
 */
#define EM_BT_CHMAP1_ADDR   (0x204F001E + EM_BT_CS_OFFSET)
#define EM_BT_CHMAP1_INDEX  0x0000000F
#define EM_BT_CHMAP1_RESET  0x0000FFFF

__INLINE uint16_t em_bt_chmap1_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_CHMAP1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_chmap1_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_CHMAP1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_CHMAP1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_CHMAP1_LSB    0
#define EM_BT_CHMAP1_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_CHMAP1_RST    0xFFFF

__INLINE uint16_t em_bt_chmap1_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_CHMAP1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_chmap1_setf(int elt_idx, uint16_t chmap1)
{
    ASSERT_ERR((((uint16_t)chmap1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_CHMAP1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)chmap1 << 0);
}

/**
 * @brief CHMAP2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               CHMAP2   0xFFFF
 * </pre>
 */
#define EM_BT_CHMAP2_ADDR   (0x204F0020 + EM_BT_CS_OFFSET)
#define EM_BT_CHMAP2_INDEX  0x00000010
#define EM_BT_CHMAP2_RESET  0x0000FFFF

__INLINE uint16_t em_bt_chmap2_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_CHMAP2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_chmap2_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_CHMAP2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_CHMAP2_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_CHMAP2_LSB    0
#define EM_BT_CHMAP2_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_CHMAP2_RST    0xFFFF

__INLINE uint16_t em_bt_chmap2_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_CHMAP2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_chmap2_setf(int elt_idx, uint16_t chmap2)
{
    ASSERT_ERR((((uint16_t)chmap2 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_CHMAP2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)chmap2 << 0);
}

/**
 * @brief CHMAP3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               CHMAP3   0xFFFF
 * </pre>
 */
#define EM_BT_CHMAP3_ADDR   (0x204F0022 + EM_BT_CS_OFFSET)
#define EM_BT_CHMAP3_INDEX  0x00000011
#define EM_BT_CHMAP3_RESET  0x0000FFFF

__INLINE uint16_t em_bt_chmap3_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_CHMAP3_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_chmap3_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_CHMAP3_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_CHMAP3_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_CHMAP3_LSB    0
#define EM_BT_CHMAP3_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_CHMAP3_RST    0xFFFF

__INLINE uint16_t em_bt_chmap3_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_CHMAP3_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_chmap3_setf(int elt_idx, uint16_t chmap3)
{
    ASSERT_ERR((((uint16_t)chmap3 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_CHMAP3_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)chmap3 << 0);
}

/**
 * @brief CHMAP4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  14:00               CHMAP4   0x7FFF
 * </pre>
 */
#define EM_BT_CHMAP4_ADDR   (0x204F0024 + EM_BT_CS_OFFSET)
#define EM_BT_CHMAP4_INDEX  0x00000012
#define EM_BT_CHMAP4_RESET  0x00007FFF

__INLINE uint16_t em_bt_chmap4_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_CHMAP4_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_chmap4_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_CHMAP4_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_CHMAP4_MASK   ((uint16_t)0x00007FFF)
#define EM_BT_CHMAP4_LSB    0
#define EM_BT_CHMAP4_WIDTH  ((uint16_t)0x0000000F)

#define EM_BT_CHMAP4_RST    0x7FFF

__INLINE uint16_t em_bt_chmap4_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_CHMAP4_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00007FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_chmap4_setf(int elt_idx, uint16_t chmap4)
{
    ASSERT_ERR((((uint16_t)chmap4 << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_BT_WR(EM_BT_CHMAP4_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)chmap4 << 0);
}

/**
 * @brief LOOPCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08                TO_NB   0x0
 *  07:00               ATT_NB   0x1
 * </pre>
 */
#define EM_BT_LOOPCNTL_ADDR   (0x204F0026 + EM_BT_CS_OFFSET)
#define EM_BT_LOOPCNTL_INDEX  0x00000013
#define EM_BT_LOOPCNTL_RESET  0x00000001

__INLINE uint16_t em_bt_loopcntl_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_loopcntl_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_TO_NB_MASK    ((uint16_t)0x0000FF00)
#define EM_BT_TO_NB_LSB     8
#define EM_BT_TO_NB_WIDTH   ((uint16_t)0x00000008)
#define EM_BT_ATT_NB_MASK   ((uint16_t)0x000000FF)
#define EM_BT_ATT_NB_LSB    0
#define EM_BT_ATT_NB_WIDTH  ((uint16_t)0x00000008)

#define EM_BT_TO_NB_RST     0x0
#define EM_BT_ATT_NB_RST    0x1

__INLINE void em_bt_loopcntl_pack(int elt_idx, uint8_t tonb, uint8_t attnb)
{
    ASSERT_ERR((((uint16_t)tonb << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)attnb << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BT_WR(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)tonb << 8) | ((uint16_t)attnb << 0));
}

__INLINE void em_bt_loopcntl_unpack(int elt_idx, uint8_t *tonb, uint8_t *attnb)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *tonb = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *attnb = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_bt_loopcntl_to_nb_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_bt_loopcntl_to_nb_setf(int elt_idx, uint8_t tonb)
{
    ASSERT_ERR((((uint16_t)tonb << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BT_WR(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)tonb << 8));
}

__INLINE uint8_t em_bt_loopcntl_att_nb_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_bt_loopcntl_att_nb_setf(int elt_idx, uint8_t attnb)
{
    ASSERT_ERR((((uint16_t)attnb << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BT_WR(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_LOOPCNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)attnb << 0));
}

/**
 * @brief MAXFRMTIME register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  12:00           MAXFRMTIME   0x0
 * </pre>
 */
#define EM_BT_MAXFRMTIME_ADDR   (0x204F0028 + EM_BT_CS_OFFSET)
#define EM_BT_MAXFRMTIME_INDEX  0x00000014
#define EM_BT_MAXFRMTIME_RESET  0x00000000

__INLINE uint16_t em_bt_maxfrmtime_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_MAXFRMTIME_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_maxfrmtime_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_MAXFRMTIME_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_MAXFRMTIME_MASK   ((uint16_t)0x00001FFF)
#define EM_BT_MAXFRMTIME_LSB    0
#define EM_BT_MAXFRMTIME_WIDTH  ((uint16_t)0x0000000D)

#define EM_BT_MAXFRMTIME_RST    0x0

__INLINE uint16_t em_bt_maxfrmtime_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_MAXFRMTIME_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00001FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_maxfrmtime_setf(int elt_idx, uint16_t maxfrmtime)
{
    ASSERT_ERR((((uint16_t)maxfrmtime << 0) & ~((uint16_t)0x00001FFF)) == 0);
    EM_BT_WR(EM_BT_MAXFRMTIME_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)maxfrmtime << 0);
}

/**
 * @brief SAM_PTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00              SAM_PTR   0x0
 * </pre>
 */
#define EM_BT_SAM_PTR_ADDR   (0x204F002A + EM_BT_CS_OFFSET)
#define EM_BT_SAM_PTR_INDEX  0x00000015
#define EM_BT_SAM_PTR_RESET  0x00000000

__INLINE uint16_t em_bt_sam_ptr_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_SAM_PTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_sam_ptr_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_SAM_PTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_SAM_PTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BT_SAM_PTR_LSB    0
#define EM_BT_SAM_PTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BT_SAM_PTR_RST    0x0

__INLINE uint16_t em_bt_sam_ptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_SAM_PTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_sam_ptr_setf(int elt_idx, uint16_t samptr)
{
    ASSERT_ERR((((uint16_t)samptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BT_WR(EM_BT_SAM_PTR_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)samptr << 0);
}

/**
 * @brief SAM_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08           SAM_LENGTH   0x0
 *  07:00           SAM_OFFSET   0x0
 * </pre>
 */
#define EM_BT_SAM_CNTL_ADDR   (0x204F002C + EM_BT_CS_OFFSET)
#define EM_BT_SAM_CNTL_INDEX  0x00000016
#define EM_BT_SAM_CNTL_RESET  0x00000000

__INLINE uint16_t em_bt_sam_cntl_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_sam_cntl_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_SAM_LENGTH_MASK   ((uint16_t)0x0000FF00)
#define EM_BT_SAM_LENGTH_LSB    8
#define EM_BT_SAM_LENGTH_WIDTH  ((uint16_t)0x00000008)
#define EM_BT_SAM_OFFSET_MASK   ((uint16_t)0x000000FF)
#define EM_BT_SAM_OFFSET_LSB    0
#define EM_BT_SAM_OFFSET_WIDTH  ((uint16_t)0x00000008)

#define EM_BT_SAM_LENGTH_RST    0x0
#define EM_BT_SAM_OFFSET_RST    0x0

__INLINE void em_bt_sam_cntl_pack(int elt_idx, uint8_t samlength, uint8_t samoffset)
{
    ASSERT_ERR((((uint16_t)samlength << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)samoffset << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BT_WR(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)samlength << 8) | ((uint16_t)samoffset << 0));
}

__INLINE void em_bt_sam_cntl_unpack(int elt_idx, uint8_t *samlength, uint8_t *samoffset)
{
    uint16_t localVal = EM_BT_RD(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *samlength = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *samoffset = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_bt_sam_cntl_sam_length_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_bt_sam_cntl_sam_length_setf(int elt_idx, uint8_t samlength)
{
    ASSERT_ERR((((uint16_t)samlength << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BT_WR(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)samlength << 8));
}

__INLINE uint8_t em_bt_sam_cntl_sam_offset_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_bt_sam_cntl_sam_offset_setf(int elt_idx, uint8_t samoffset)
{
    ASSERT_ERR((((uint16_t)samoffset << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BT_WR(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_SAM_CNTL_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)samoffset << 0));
}

/**
 * @brief SK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                   SK   0x0
 * </pre>
 */
#define EM_BT_SK_ADDR   (0x204F002E + EM_BT_CS_OFFSET)
#define EM_BT_SK_INDEX  0x00000017
#define EM_BT_SK_RESET  0x00000000
#define EM_BT_SK_COUNT  8

__INLINE uint16_t em_bt_sk_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    return EM_BT_RD(EM_BT_SK_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2);
}

__INLINE void em_bt_sk_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 7);
    EM_BT_WR(EM_BT_SK_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BT_SK_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_SK_LSB    0
#define EM_BT_SK_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_SK_RST    0x0

__INLINE uint16_t em_bt_sk_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    uint16_t localVal = EM_BT_RD(EM_BT_SK_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_sk_setf(int elt_idx, int reg_idx, uint16_t sk)
{
    ASSERT_ERR(reg_idx <= 7);
    ASSERT_ERR((((uint16_t)sk << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_SK_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2, (uint16_t)sk << 0);
}

/**
 * @brief IV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                   IV   0x0
 * </pre>
 */
#define EM_BT_IV_ADDR   (0x204F003E + EM_BT_CS_OFFSET)
#define EM_BT_IV_INDEX  0x0000001F
#define EM_BT_IV_RESET  0x00000000
#define EM_BT_IV_COUNT  4

__INLINE uint16_t em_bt_iv_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 3);
    return EM_BT_RD(EM_BT_IV_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2);
}

__INLINE void em_bt_iv_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 3);
    EM_BT_WR(EM_BT_IV_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BT_IV_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_IV_LSB    0
#define EM_BT_IV_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_IV_RST    0x0

__INLINE uint16_t em_bt_iv_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 3);
    uint16_t localVal = EM_BT_RD(EM_BT_IV_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_iv_setf(int elt_idx, int reg_idx, uint16_t iv)
{
    ASSERT_ERR(reg_idx <= 3);
    ASSERT_ERR((((uint16_t)iv << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_IV_ADDR + elt_idx * REG_EM_BT_CS_SIZE + reg_idx * 2, (uint16_t)iv << 0);
}

/**
 * @brief TXCCMPLDCNT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         TXCCMPLDCNT0   0x0
 * </pre>
 */
#define EM_BT_TXCCMPLDCNT0_ADDR   (0x204F0046 + EM_BT_CS_OFFSET)
#define EM_BT_TXCCMPLDCNT0_INDEX  0x00000023
#define EM_BT_TXCCMPLDCNT0_RESET  0x00000000

__INLINE uint16_t em_bt_txccmpldcnt0_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_txccmpldcnt0_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_TXCCMPLDCNT0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_TXCCMPLDCNT0_LSB    0
#define EM_BT_TXCCMPLDCNT0_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_TXCCMPLDCNT0_RST    0x0

__INLINE uint16_t em_bt_txccmpldcnt0_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_txccmpldcnt0_setf(int elt_idx, uint16_t txccmpldcnt0)
{
    ASSERT_ERR((((uint16_t)txccmpldcnt0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_TXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)txccmpldcnt0 << 0);
}

/**
 * @brief TXCCMPLDCNT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         TXCCMPLDCNT1   0x0
 * </pre>
 */
#define EM_BT_TXCCMPLDCNT1_ADDR   (0x204F0048 + EM_BT_CS_OFFSET)
#define EM_BT_TXCCMPLDCNT1_INDEX  0x00000024
#define EM_BT_TXCCMPLDCNT1_RESET  0x00000000

__INLINE uint16_t em_bt_txccmpldcnt1_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_txccmpldcnt1_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_TXCCMPLDCNT1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_TXCCMPLDCNT1_LSB    0
#define EM_BT_TXCCMPLDCNT1_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_TXCCMPLDCNT1_RST    0x0

__INLINE uint16_t em_bt_txccmpldcnt1_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_txccmpldcnt1_setf(int elt_idx, uint16_t txccmpldcnt1)
{
    ASSERT_ERR((((uint16_t)txccmpldcnt1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_TXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)txccmpldcnt1 << 0);
}

/**
 * @brief TXCCMPLDCNT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00         TXCCMPLDCNT2   0x0
 * </pre>
 */
#define EM_BT_TXCCMPLDCNT2_ADDR   (0x204F004A + EM_BT_CS_OFFSET)
#define EM_BT_TXCCMPLDCNT2_INDEX  0x00000025
#define EM_BT_TXCCMPLDCNT2_RESET  0x00000000

__INLINE uint16_t em_bt_txccmpldcnt2_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_txccmpldcnt2_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_TXCCMPLDCNT2_MASK   ((uint16_t)0x0000000F)
#define EM_BT_TXCCMPLDCNT2_LSB    0
#define EM_BT_TXCCMPLDCNT2_WIDTH  ((uint16_t)0x00000004)

#define EM_BT_TXCCMPLDCNT2_RST    0x0

__INLINE uint8_t em_bt_txccmpldcnt2_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000000F)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_txccmpldcnt2_setf(int elt_idx, uint8_t txccmpldcnt2)
{
    ASSERT_ERR((((uint16_t)txccmpldcnt2 << 0) & ~((uint16_t)0x0000000F)) == 0);
    EM_BT_WR(EM_BT_TXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)txccmpldcnt2 << 0);
}

/**
 * @brief RXCCMPLDCNT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         RXCCMPLDCNT0   0x0
 * </pre>
 */
#define EM_BT_RXCCMPLDCNT0_ADDR   (0x204F004C + EM_BT_CS_OFFSET)
#define EM_BT_RXCCMPLDCNT0_INDEX  0x00000026
#define EM_BT_RXCCMPLDCNT0_RESET  0x00000000

__INLINE uint16_t em_bt_rxccmpldcnt0_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_rxccmpldcnt0_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_RXCCMPLDCNT0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_RXCCMPLDCNT0_LSB    0
#define EM_BT_RXCCMPLDCNT0_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_RXCCMPLDCNT0_RST    0x0

__INLINE uint16_t em_bt_rxccmpldcnt0_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_rxccmpldcnt0_setf(int elt_idx, uint16_t rxccmpldcnt0)
{
    ASSERT_ERR((((uint16_t)rxccmpldcnt0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_RXCCMPLDCNT0_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)rxccmpldcnt0 << 0);
}

/**
 * @brief RXCCMPLDCNT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         RXCCMPLDCNT1   0x0
 * </pre>
 */
#define EM_BT_RXCCMPLDCNT1_ADDR   (0x204F004E + EM_BT_CS_OFFSET)
#define EM_BT_RXCCMPLDCNT1_INDEX  0x00000027
#define EM_BT_RXCCMPLDCNT1_RESET  0x00000000

__INLINE uint16_t em_bt_rxccmpldcnt1_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_rxccmpldcnt1_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_RXCCMPLDCNT1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_RXCCMPLDCNT1_LSB    0
#define EM_BT_RXCCMPLDCNT1_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_RXCCMPLDCNT1_RST    0x0

__INLINE uint16_t em_bt_rxccmpldcnt1_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_rxccmpldcnt1_setf(int elt_idx, uint16_t rxccmpldcnt1)
{
    ASSERT_ERR((((uint16_t)rxccmpldcnt1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BT_WR(EM_BT_RXCCMPLDCNT1_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)rxccmpldcnt1 << 0);
}

/**
 * @brief RXCCMPLDCNT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00         RXCCMPLDCNT2   0x0
 * </pre>
 */
#define EM_BT_RXCCMPLDCNT2_ADDR   (0x204F0050 + EM_BT_CS_OFFSET)
#define EM_BT_RXCCMPLDCNT2_INDEX  0x00000028
#define EM_BT_RXCCMPLDCNT2_RESET  0x00000000

__INLINE uint16_t em_bt_rxccmpldcnt2_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_rxccmpldcnt2_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_RXCCMPLDCNT2_MASK   ((uint16_t)0x0000000F)
#define EM_BT_RXCCMPLDCNT2_LSB    0
#define EM_BT_RXCCMPLDCNT2_WIDTH  ((uint16_t)0x00000004)

#define EM_BT_RXCCMPLDCNT2_RST    0x0

__INLINE uint8_t em_bt_rxccmpldcnt2_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000000F)) == 0);
    return (localVal >> 0);
}

__INLINE void em_bt_rxccmpldcnt2_setf(int elt_idx, uint8_t rxccmpldcnt2)
{
    ASSERT_ERR((((uint16_t)rxccmpldcnt2 << 0) & ~((uint16_t)0x0000000F)) == 0);
    EM_BT_WR(EM_BT_RXCCMPLDCNT2_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (uint16_t)rxccmpldcnt2 << 0);
}

/**
 * @brief RXCLKN0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              RXCLKN0   0x0
 * </pre>
 */
#define EM_BT_RXCLKN0_ADDR   (0x204F0052 + EM_BT_CS_OFFSET)
#define EM_BT_RXCLKN0_INDEX  0x00000029
#define EM_BT_RXCLKN0_RESET  0x00000000

__INLINE uint16_t em_bt_rxclkn0_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXCLKN0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

// field definitions
#define EM_BT_RXCLKN0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BT_RXCLKN0_LSB    0
#define EM_BT_RXCLKN0_WIDTH  ((uint16_t)0x00000010)

#define EM_BT_RXCLKN0_RST    0x0

__INLINE uint16_t em_bt_rxclkn0_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCLKN0_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RXCLKN1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:00              RXCLKN1   0x0
 * </pre>
 */
#define EM_BT_RXCLKN1_ADDR   (0x204F0054 + EM_BT_CS_OFFSET)
#define EM_BT_RXCLKN1_INDEX  0x0000002A
#define EM_BT_RXCLKN1_RESET  0x00000000

__INLINE uint16_t em_bt_rxclkn1_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXCLKN1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

// field definitions
#define EM_BT_RXCLKN1_MASK   ((uint16_t)0x00000FFF)
#define EM_BT_RXCLKN1_LSB    0
#define EM_BT_RXCLKN1_WIDTH  ((uint16_t)0x0000000C)

#define EM_BT_RXCLKN1_RST    0x0

__INLINE uint16_t em_bt_rxclkn1_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXCLKN1_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00000FFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RXBIT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               FRRXOK   0
 *  09:00                RXBIT   0x0
 * </pre>
 */
#define EM_BT_RXBIT_ADDR   (0x204F0056 + EM_BT_CS_OFFSET)
#define EM_BT_RXBIT_INDEX  0x0000002B
#define EM_BT_RXBIT_RESET  0x00000000

__INLINE uint16_t em_bt_rxbit_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXBIT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

// field definitions
#define EM_BT_FRRXOK_BIT    ((uint16_t)0x00008000)
#define EM_BT_FRRXOK_POS    15
#define EM_BT_RXBIT_MASK    ((uint16_t)0x000003FF)
#define EM_BT_RXBIT_LSB     0
#define EM_BT_RXBIT_WIDTH   ((uint16_t)0x0000000A)

#define EM_BT_FRRXOK_RST    0x0
#define EM_BT_RXBIT_RST     0x0

__INLINE void em_bt_rxbit_unpack(int elt_idx, uint8_t *frrxok, uint16_t *rxbit)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXBIT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *frrxok = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxbit = (localVal & ((uint16_t)0x000003FF)) >> 0;
}

__INLINE uint8_t em_bt_rxbit_frrxok_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXBIT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE uint16_t em_bt_rxbit_rxbit_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXBIT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x000003FF)) >> 0);
}

/**
 * @brief TXDESCCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  12:08       AUDIOTXDESCCNT   0x0
 *  04:00         ACLTXDESCCNT   0x0
 * </pre>
 */
#define EM_BT_TXDESCCNT_ADDR   (0x204F0058 + EM_BT_CS_OFFSET)
#define EM_BT_TXDESCCNT_INDEX  0x0000002C
#define EM_BT_TXDESCCNT_RESET  0x00000000

__INLINE uint16_t em_bt_txdesccnt_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_txdesccnt_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_AUDIOTXDESCCNT_MASK   ((uint16_t)0x00001F00)
#define EM_BT_AUDIOTXDESCCNT_LSB    8
#define EM_BT_AUDIOTXDESCCNT_WIDTH  ((uint16_t)0x00000005)
#define EM_BT_ACLTXDESCCNT_MASK     ((uint16_t)0x0000001F)
#define EM_BT_ACLTXDESCCNT_LSB      0
#define EM_BT_ACLTXDESCCNT_WIDTH    ((uint16_t)0x00000005)

#define EM_BT_AUDIOTXDESCCNT_RST    0x0
#define EM_BT_ACLTXDESCCNT_RST      0x0

__INLINE void em_bt_txdesccnt_pack(int elt_idx, uint8_t audiotxdesccnt, uint8_t acltxdesccnt)
{
    ASSERT_ERR((((uint16_t)audiotxdesccnt << 8) & ~((uint16_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint16_t)acltxdesccnt << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)audiotxdesccnt << 8) | ((uint16_t)acltxdesccnt << 0));
}

__INLINE void em_bt_txdesccnt_unpack(int elt_idx, uint8_t *audiotxdesccnt, uint8_t *acltxdesccnt)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *audiotxdesccnt = (localVal & ((uint16_t)0x00001F00)) >> 8;
    *acltxdesccnt = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_bt_txdesccnt_audiotxdesccnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001F00)) >> 8);
}

__INLINE void em_bt_txdesccnt_audiotxdesccnt_setf(int elt_idx, uint8_t audiotxdesccnt)
{
    ASSERT_ERR((((uint16_t)audiotxdesccnt << 8) & ~((uint16_t)0x00001F00)) == 0);
    EM_BT_WR(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00001F00)) | ((uint16_t)audiotxdesccnt << 8));
}

__INLINE uint8_t em_bt_txdesccnt_acltxdesccnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_bt_txdesccnt_acltxdesccnt_setf(int elt_idx, uint8_t acltxdesccnt)
{
    ASSERT_ERR((((uint16_t)acltxdesccnt << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_TXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)acltxdesccnt << 0));
}

/**
 * @brief RXDESCCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  12:08       AUDIORXDESCCNT   0x0
 *  04:00         ACLRXDESCCNT   0x0
 * </pre>
 */
#define EM_BT_RXDESCCNT_ADDR   (0x204F005A + EM_BT_CS_OFFSET)
#define EM_BT_RXDESCCNT_INDEX  0x0000002D
#define EM_BT_RXDESCCNT_RESET  0x00000000

__INLINE uint16_t em_bt_rxdesccnt_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_rxdesccnt_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_AUDIORXDESCCNT_MASK   ((uint16_t)0x00001F00)
#define EM_BT_AUDIORXDESCCNT_LSB    8
#define EM_BT_AUDIORXDESCCNT_WIDTH  ((uint16_t)0x00000005)
#define EM_BT_ACLRXDESCCNT_MASK     ((uint16_t)0x0000001F)
#define EM_BT_ACLRXDESCCNT_LSB      0
#define EM_BT_ACLRXDESCCNT_WIDTH    ((uint16_t)0x00000005)

#define EM_BT_AUDIORXDESCCNT_RST    0x0
#define EM_BT_ACLRXDESCCNT_RST      0x0

__INLINE void em_bt_rxdesccnt_pack(int elt_idx, uint8_t audiorxdesccnt, uint8_t aclrxdesccnt)
{
    ASSERT_ERR((((uint16_t)audiorxdesccnt << 8) & ~((uint16_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint16_t)aclrxdesccnt << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)audiorxdesccnt << 8) | ((uint16_t)aclrxdesccnt << 0));
}

__INLINE void em_bt_rxdesccnt_unpack(int elt_idx, uint8_t *audiorxdesccnt, uint8_t *aclrxdesccnt)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *audiorxdesccnt = (localVal & ((uint16_t)0x00001F00)) >> 8;
    *aclrxdesccnt = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_bt_rxdesccnt_audiorxdesccnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001F00)) >> 8);
}

__INLINE void em_bt_rxdesccnt_audiorxdesccnt_setf(int elt_idx, uint8_t audiorxdesccnt)
{
    ASSERT_ERR((((uint16_t)audiorxdesccnt << 8) & ~((uint16_t)0x00001F00)) == 0);
    EM_BT_WR(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00001F00)) | ((uint16_t)audiorxdesccnt << 8));
}

__INLINE uint8_t em_bt_rxdesccnt_aclrxdesccnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_bt_rxdesccnt_aclrxdesccnt_setf(int elt_idx, uint8_t aclrxdesccnt)
{
    ASSERT_ERR((((uint16_t)aclrxdesccnt << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BT_WR(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_RXDESCCNT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)aclrxdesccnt << 0));
}

/**
 * @brief ACLTXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15                TXTOG   0
 *     14              TXFLUSH   0
 *     13                 FCRC   0
 *     12                 FNAK   0
 *     11              RSWITCH   0
 *     10              WAITACK   0
 *     07            LAST_NULL   0
 *     01            LASTTXACK   0
 *     00           LASTTXSEQN   0
 * </pre>
 */
#define EM_BT_ACLTXSTAT_ADDR   (0x204F005C + EM_BT_CS_OFFSET)
#define EM_BT_ACLTXSTAT_INDEX  0x0000002E
#define EM_BT_ACLTXSTAT_RESET  0x00000000

__INLINE uint16_t em_bt_acltxstat_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_acltxstat_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_TXTOG_BIT         ((uint16_t)0x00008000)
#define EM_BT_TXTOG_POS         15
#define EM_BT_TXFLUSH_BIT       ((uint16_t)0x00004000)
#define EM_BT_TXFLUSH_POS       14
#define EM_BT_FCRC_BIT          ((uint16_t)0x00002000)
#define EM_BT_FCRC_POS          13
#define EM_BT_FNAK_BIT          ((uint16_t)0x00001000)
#define EM_BT_FNAK_POS          12
#define EM_BT_RSWITCH_BIT       ((uint16_t)0x00000800)
#define EM_BT_RSWITCH_POS       11
#define EM_BT_WAITACK_BIT       ((uint16_t)0x00000400)
#define EM_BT_WAITACK_POS       10
#define EM_BT_LAST_NULL_BIT     ((uint16_t)0x00000080)
#define EM_BT_LAST_NULL_POS     7
#define EM_BT_LASTTXACK_BIT     ((uint16_t)0x00000002)
#define EM_BT_LASTTXACK_POS     1
#define EM_BT_LASTTXSEQN_BIT    ((uint16_t)0x00000001)
#define EM_BT_LASTTXSEQN_POS    0

#define EM_BT_TXTOG_RST         0x0
#define EM_BT_TXFLUSH_RST       0x0
#define EM_BT_FCRC_RST          0x0
#define EM_BT_FNAK_RST          0x0
#define EM_BT_RSWITCH_RST       0x0
#define EM_BT_WAITACK_RST       0x0
#define EM_BT_LAST_NULL_RST     0x0
#define EM_BT_LASTTXACK_RST     0x0
#define EM_BT_LASTTXSEQN_RST    0x0

__INLINE void em_bt_acltxstat_pack(int elt_idx, uint8_t txtog, uint8_t txflush, uint8_t fcrc, uint8_t fnak, uint8_t rswitch, uint8_t waitack, uint8_t lastnull, uint8_t lasttxack, uint8_t lasttxseqn)
{
    ASSERT_ERR((((uint16_t)txtog << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)txflush << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)fcrc << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)fnak << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)rswitch << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)waitack << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)lastnull << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)lasttxack << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)lasttxseqn << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)txtog << 15) | ((uint16_t)txflush << 14) | ((uint16_t)fcrc << 13) | ((uint16_t)fnak << 12) | ((uint16_t)rswitch << 11) | ((uint16_t)waitack << 10) | ((uint16_t)lastnull << 7) | ((uint16_t)lasttxack << 1) | ((uint16_t)lasttxseqn << 0));
}

__INLINE void em_bt_acltxstat_unpack(int elt_idx, uint8_t *txtog, uint8_t *txflush, uint8_t *fcrc, uint8_t *fnak, uint8_t *rswitch, uint8_t *waitack, uint8_t *lastnull, uint8_t *lasttxack, uint8_t *lasttxseqn)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *txtog = (localVal & ((uint16_t)0x00008000)) >> 15;
    *txflush = (localVal & ((uint16_t)0x00004000)) >> 14;
    *fcrc = (localVal & ((uint16_t)0x00002000)) >> 13;
    *fnak = (localVal & ((uint16_t)0x00001000)) >> 12;
    *rswitch = (localVal & ((uint16_t)0x00000800)) >> 11;
    *waitack = (localVal & ((uint16_t)0x00000400)) >> 10;
    *lastnull = (localVal & ((uint16_t)0x00000080)) >> 7;
    *lasttxack = (localVal & ((uint16_t)0x00000002)) >> 1;
    *lasttxseqn = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_bt_acltxstat_txtog_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_acltxstat_txtog_setf(int elt_idx, uint8_t txtog)
{
    ASSERT_ERR((((uint16_t)txtog << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)txtog << 15));
}

__INLINE uint8_t em_bt_acltxstat_txflush_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_bt_acltxstat_txflush_setf(int elt_idx, uint8_t txflush)
{
    ASSERT_ERR((((uint16_t)txflush << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)txflush << 14));
}

__INLINE uint8_t em_bt_acltxstat_fcrc_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_bt_acltxstat_fcrc_setf(int elt_idx, uint8_t fcrc)
{
    ASSERT_ERR((((uint16_t)fcrc << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)fcrc << 13));
}

__INLINE uint8_t em_bt_acltxstat_fnak_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_bt_acltxstat_fnak_setf(int elt_idx, uint8_t fnak)
{
    ASSERT_ERR((((uint16_t)fnak << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)fnak << 12));
}

__INLINE uint8_t em_bt_acltxstat_rswitch_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_bt_acltxstat_rswitch_setf(int elt_idx, uint8_t rswitch)
{
    ASSERT_ERR((((uint16_t)rswitch << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)rswitch << 11));
}

__INLINE uint8_t em_bt_acltxstat_waitack_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_bt_acltxstat_waitack_setf(int elt_idx, uint8_t waitack)
{
    ASSERT_ERR((((uint16_t)waitack << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)waitack << 10));
}

__INLINE uint8_t em_bt_acltxstat_last_null_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_bt_acltxstat_last_null_setf(int elt_idx, uint8_t lastnull)
{
    ASSERT_ERR((((uint16_t)lastnull << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)lastnull << 7));
}

__INLINE uint8_t em_bt_acltxstat_lasttxack_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_bt_acltxstat_lasttxack_setf(int elt_idx, uint8_t lasttxack)
{
    ASSERT_ERR((((uint16_t)lasttxack << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)lasttxack << 1));
}

__INLINE uint8_t em_bt_acltxstat_lasttxseqn_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_bt_acltxstat_lasttxseqn_setf(int elt_idx, uint8_t lasttxseqn)
{
    ASSERT_ERR((((uint16_t)lasttxseqn << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BT_WR(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLTXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)lasttxseqn << 0));
}

/**
 * @brief ACLRXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15          RXBUFF_FULL   0
 *     14           RXBFMICERR   0
 *     13                 RXID   0
 *     12                RXACL   0
 *     11              RXAUDIO   0
 *     10                RXLMP   0
 *     02           LASTRXFLOW   0
 *     01            LASTRXACK   0
 *     00           LASTRXSEQN   0
 * </pre>
 */
#define EM_BT_ACLRXSTAT_ADDR   (0x204F005E + EM_BT_CS_OFFSET)
#define EM_BT_ACLRXSTAT_INDEX  0x0000002F
#define EM_BT_ACLRXSTAT_RESET  0x00000000

__INLINE uint16_t em_bt_aclrxstat_get(int elt_idx)
{
    return EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
}

__INLINE void em_bt_aclrxstat_set(int elt_idx, uint16_t value)
{
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, value);
}

// field definitions
#define EM_BT_RXBUFF_FULL_BIT    ((uint16_t)0x00008000)
#define EM_BT_RXBUFF_FULL_POS    15
#define EM_BT_RXBFMICERR_BIT     ((uint16_t)0x00004000)
#define EM_BT_RXBFMICERR_POS     14
#define EM_BT_RXID_BIT           ((uint16_t)0x00002000)
#define EM_BT_RXID_POS           13
#define EM_BT_RXACL_BIT          ((uint16_t)0x00001000)
#define EM_BT_RXACL_POS          12
#define EM_BT_RXAUDIO_BIT        ((uint16_t)0x00000800)
#define EM_BT_RXAUDIO_POS        11
#define EM_BT_RXLMP_BIT          ((uint16_t)0x00000400)
#define EM_BT_RXLMP_POS          10
#define EM_BT_LASTRXFLOW_BIT     ((uint16_t)0x00000004)
#define EM_BT_LASTRXFLOW_POS     2
#define EM_BT_LASTRXACK_BIT      ((uint16_t)0x00000002)
#define EM_BT_LASTRXACK_POS      1
#define EM_BT_LASTRXSEQN_BIT     ((uint16_t)0x00000001)
#define EM_BT_LASTRXSEQN_POS     0

#define EM_BT_RXBUFF_FULL_RST    0x0
#define EM_BT_RXBFMICERR_RST     0x0
#define EM_BT_RXID_RST           0x0
#define EM_BT_RXACL_RST          0x0
#define EM_BT_RXAUDIO_RST        0x0
#define EM_BT_RXLMP_RST          0x0
#define EM_BT_LASTRXFLOW_RST     0x0
#define EM_BT_LASTRXACK_RST      0x0
#define EM_BT_LASTRXSEQN_RST     0x0

__INLINE void em_bt_aclrxstat_pack(int elt_idx, uint8_t rxbufffull, uint8_t rxbfmicerr, uint8_t rxid, uint8_t rxacl, uint8_t rxaudio, uint8_t rxlmp, uint8_t lastrxflow, uint8_t lastrxack, uint8_t lastrxseqn)
{
    ASSERT_ERR((((uint16_t)rxbufffull << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)rxbfmicerr << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)rxid << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)rxacl << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)rxaudio << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)rxlmp << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)lastrxflow << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)lastrxack << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)lastrxseqn << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, ((uint16_t)rxbufffull << 15) | ((uint16_t)rxbfmicerr << 14) | ((uint16_t)rxid << 13) | ((uint16_t)rxacl << 12) | ((uint16_t)rxaudio << 11) | ((uint16_t)rxlmp << 10) | ((uint16_t)lastrxflow << 2) | ((uint16_t)lastrxack << 1) | ((uint16_t)lastrxseqn << 0));
}

__INLINE void em_bt_aclrxstat_unpack(int elt_idx, uint8_t *rxbufffull, uint8_t *rxbfmicerr, uint8_t *rxid, uint8_t *rxacl, uint8_t *rxaudio, uint8_t *rxlmp, uint8_t *lastrxflow, uint8_t *lastrxack, uint8_t *lastrxseqn)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);

    *rxbufffull = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxbfmicerr = (localVal & ((uint16_t)0x00004000)) >> 14;
    *rxid = (localVal & ((uint16_t)0x00002000)) >> 13;
    *rxacl = (localVal & ((uint16_t)0x00001000)) >> 12;
    *rxaudio = (localVal & ((uint16_t)0x00000800)) >> 11;
    *rxlmp = (localVal & ((uint16_t)0x00000400)) >> 10;
    *lastrxflow = (localVal & ((uint16_t)0x00000004)) >> 2;
    *lastrxack = (localVal & ((uint16_t)0x00000002)) >> 1;
    *lastrxseqn = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_bt_aclrxstat_rxbuff_full_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_bt_aclrxstat_rxbuff_full_setf(int elt_idx, uint8_t rxbufffull)
{
    ASSERT_ERR((((uint16_t)rxbufffull << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxbufffull << 15));
}

__INLINE uint8_t em_bt_aclrxstat_rxbfmicerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_bt_aclrxstat_rxbfmicerr_setf(int elt_idx, uint8_t rxbfmicerr)
{
    ASSERT_ERR((((uint16_t)rxbfmicerr << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)rxbfmicerr << 14));
}

__INLINE uint8_t em_bt_aclrxstat_rxid_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_bt_aclrxstat_rxid_setf(int elt_idx, uint8_t rxid)
{
    ASSERT_ERR((((uint16_t)rxid << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)rxid << 13));
}

__INLINE uint8_t em_bt_aclrxstat_rxacl_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_bt_aclrxstat_rxacl_setf(int elt_idx, uint8_t rxacl)
{
    ASSERT_ERR((((uint16_t)rxacl << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)rxacl << 12));
}

__INLINE uint8_t em_bt_aclrxstat_rxaudio_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_bt_aclrxstat_rxaudio_setf(int elt_idx, uint8_t rxaudio)
{
    ASSERT_ERR((((uint16_t)rxaudio << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)rxaudio << 11));
}

__INLINE uint8_t em_bt_aclrxstat_rxlmp_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_bt_aclrxstat_rxlmp_setf(int elt_idx, uint8_t rxlmp)
{
    ASSERT_ERR((((uint16_t)rxlmp << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)rxlmp << 10));
}

__INLINE uint8_t em_bt_aclrxstat_lastrxflow_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_bt_aclrxstat_lastrxflow_setf(int elt_idx, uint8_t lastrxflow)
{
    ASSERT_ERR((((uint16_t)lastrxflow << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)lastrxflow << 2));
}

__INLINE uint8_t em_bt_aclrxstat_lastrxack_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_bt_aclrxstat_lastrxack_setf(int elt_idx, uint8_t lastrxack)
{
    ASSERT_ERR((((uint16_t)lastrxack << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)lastrxack << 1));
}

__INLINE uint8_t em_bt_aclrxstat_lastrxseqn_getf(int elt_idx)
{
    uint16_t localVal = EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_bt_aclrxstat_lastrxseqn_setf(int elt_idx, uint8_t lastrxseqn)
{
    ASSERT_ERR((((uint16_t)lastrxseqn << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BT_WR(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE, (EM_BT_RD(EM_BT_ACLRXSTAT_ADDR + elt_idx * REG_EM_BT_CS_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)lastrxseqn << 0));
}


#endif // _REG_EM_BT_CS_H_

