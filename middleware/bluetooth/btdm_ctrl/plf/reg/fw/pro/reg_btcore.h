#ifndef _REG_BTCORE_H_
#define _REG_BTCORE_H_

#include <stdint.h>
#include "_reg_btcore.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_BTCORE_COUNT 169

#define REG_BTCORE_DECODING_MASK 0x000003FF

/**
 * @brief RWBTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31      MASTER_SOFT_RST   0
 *     30    MASTER_TGSOFT_RST   0
 *     29         REG_SOFT_RST   0
 *     28   RADIOCNTL_SOFT_RST   0
 *     27            SWINT_REQ   0
 *     23           SCAN_ABORT   0
 *     22         RFTEST_ABORT   0
 *     21        PAGEINQ_ABORT   0
 *     20          SNIFF_ABORT   0
 *     19           LMPFLOWDSB   0
 *     18             CRYPTDSB   0
 *     17               CRCDSB   0
 *     16              WHITDSB   0
 *     15               HOPDSB   0
 *     14              FLOWDSB   0
 *     13              ARQNDSB   0
 *     12              SEQNDSB   0
 *     11          CX_TXBSYENA   0
 *     10          CX_RXBSYENA   0
 *     09           CX_DNABORT   0
 *     08               RWBTEN   0
 *  05:00             NWINSIZE   0xB
 * </pre>
 */
#define BT_RWBTCNTL_ADDR   0x50090400
#define BT_RWBTCNTL_OFFSET 0x00000000
#define BT_RWBTCNTL_INDEX  0x00000000
#define BT_RWBTCNTL_RESET  0x0000000B

__INLINE uint32_t bt_rwbtcntl_get(void)
{
    return REG_BT_RD(BT_RWBTCNTL_ADDR);
}

__INLINE void bt_rwbtcntl_set(uint32_t value)
{
    REG_BT_WR(BT_RWBTCNTL_ADDR, value);
}

// field definitions
#define BT_MASTER_SOFT_RST_BIT       ((uint32_t)0x80000000)
#define BT_MASTER_SOFT_RST_POS       31
#define BT_MASTER_TGSOFT_RST_BIT     ((uint32_t)0x40000000)
#define BT_MASTER_TGSOFT_RST_POS     30
#define BT_REG_SOFT_RST_BIT          ((uint32_t)0x20000000)
#define BT_REG_SOFT_RST_POS          29
#define BT_RADIOCNTL_SOFT_RST_BIT    ((uint32_t)0x10000000)
#define BT_RADIOCNTL_SOFT_RST_POS    28
#define BT_SWINT_REQ_BIT             ((uint32_t)0x08000000)
#define BT_SWINT_REQ_POS             27
#define BT_SCAN_ABORT_BIT            ((uint32_t)0x00800000)
#define BT_SCAN_ABORT_POS            23
#define BT_RFTEST_ABORT_BIT          ((uint32_t)0x00400000)
#define BT_RFTEST_ABORT_POS          22
#define BT_PAGEINQ_ABORT_BIT         ((uint32_t)0x00200000)
#define BT_PAGEINQ_ABORT_POS         21
#define BT_SNIFF_ABORT_BIT           ((uint32_t)0x00100000)
#define BT_SNIFF_ABORT_POS           20
#define BT_LMPFLOWDSB_BIT            ((uint32_t)0x00080000)
#define BT_LMPFLOWDSB_POS            19
#define BT_CRYPTDSB_BIT              ((uint32_t)0x00040000)
#define BT_CRYPTDSB_POS              18
#define BT_CRCDSB_BIT                ((uint32_t)0x00020000)
#define BT_CRCDSB_POS                17
#define BT_WHITDSB_BIT               ((uint32_t)0x00010000)
#define BT_WHITDSB_POS               16
#define BT_HOPDSB_BIT                ((uint32_t)0x00008000)
#define BT_HOPDSB_POS                15
#define BT_FLOWDSB_BIT               ((uint32_t)0x00004000)
#define BT_FLOWDSB_POS               14
#define BT_ARQNDSB_BIT               ((uint32_t)0x00002000)
#define BT_ARQNDSB_POS               13
#define BT_SEQNDSB_BIT               ((uint32_t)0x00001000)
#define BT_SEQNDSB_POS               12
#define BT_CX_TXBSYENA_BIT           ((uint32_t)0x00000800)
#define BT_CX_TXBSYENA_POS           11
#define BT_CX_RXBSYENA_BIT           ((uint32_t)0x00000400)
#define BT_CX_RXBSYENA_POS           10
#define BT_CX_DNABORT_BIT            ((uint32_t)0x00000200)
#define BT_CX_DNABORT_POS            9
#define BT_RWBTEN_BIT                ((uint32_t)0x00000100)
#define BT_RWBTEN_POS                8
#define BT_NWINSIZE_MASK             ((uint32_t)0x0000003F)
#define BT_NWINSIZE_LSB              0
#define BT_NWINSIZE_WIDTH            ((uint32_t)0x00000006)

#define BT_MASTER_SOFT_RST_RST       0x0
#define BT_MASTER_TGSOFT_RST_RST     0x0
#define BT_REG_SOFT_RST_RST          0x0
#define BT_RADIOCNTL_SOFT_RST_RST    0x0
#define BT_SWINT_REQ_RST             0x0
#define BT_SCAN_ABORT_RST            0x0
#define BT_RFTEST_ABORT_RST          0x0
#define BT_PAGEINQ_ABORT_RST         0x0
#define BT_SNIFF_ABORT_RST           0x0
#define BT_LMPFLOWDSB_RST            0x0
#define BT_CRYPTDSB_RST              0x0
#define BT_CRCDSB_RST                0x0
#define BT_WHITDSB_RST               0x0
#define BT_HOPDSB_RST                0x0
#define BT_FLOWDSB_RST               0x0
#define BT_ARQNDSB_RST               0x0
#define BT_SEQNDSB_RST               0x0
#define BT_CX_TXBSYENA_RST           0x0
#define BT_CX_RXBSYENA_RST           0x0
#define BT_CX_DNABORT_RST            0x0
#define BT_RWBTEN_RST                0x0
#define BT_NWINSIZE_RST              0xB

__INLINE void bt_rwbtcntl_pack(uint8_t mastersoftrst, uint8_t mastertgsoftrst, uint8_t regsoftrst, uint8_t radiocntlsoftrst, uint8_t swintreq, uint8_t scanabort, uint8_t rftestabort, uint8_t pageinqabort, uint8_t sniffabort, uint8_t lmpflowdsb, uint8_t cryptdsb, uint8_t crcdsb, uint8_t whitdsb, uint8_t hopdsb, uint8_t flowdsb, uint8_t arqndsb, uint8_t seqndsb, uint8_t cxtxbsyena, uint8_t cxrxbsyena, uint8_t cxdnabort, uint8_t rwbten, uint8_t nwinsize)
{
    ASSERT_ERR((((uint32_t)mastersoftrst << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)mastertgsoftrst << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)regsoftrst << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)radiocntlsoftrst << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)swintreq << 27) & ~((uint32_t)0x08000000)) == 0);
    ASSERT_ERR((((uint32_t)scanabort << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)rftestabort << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)pageinqabort << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)sniffabort << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)lmpflowdsb << 19) & ~((uint32_t)0x00080000)) == 0);
    ASSERT_ERR((((uint32_t)cryptdsb << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)crcdsb << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)whitdsb << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)hopdsb << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)flowdsb << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)arqndsb << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)seqndsb << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)cxtxbsyena << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)cxrxbsyena << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)cxdnabort << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)rwbten << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)nwinsize << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, ((uint32_t)mastersoftrst << 31) | ((uint32_t)mastertgsoftrst << 30) | ((uint32_t)regsoftrst << 29) | ((uint32_t)radiocntlsoftrst << 28) | ((uint32_t)swintreq << 27) | ((uint32_t)scanabort << 23) | ((uint32_t)rftestabort << 22) | ((uint32_t)pageinqabort << 21) | ((uint32_t)sniffabort << 20) | ((uint32_t)lmpflowdsb << 19) | ((uint32_t)cryptdsb << 18) | ((uint32_t)crcdsb << 17) | ((uint32_t)whitdsb << 16) | ((uint32_t)hopdsb << 15) | ((uint32_t)flowdsb << 14) | ((uint32_t)arqndsb << 13) | ((uint32_t)seqndsb << 12) | ((uint32_t)cxtxbsyena << 11) | ((uint32_t)cxrxbsyena << 10) | ((uint32_t)cxdnabort << 9) | ((uint32_t)rwbten << 8) | ((uint32_t)nwinsize << 0));
}

__INLINE void bt_rwbtcntl_unpack(uint8_t *mastersoftrst, uint8_t *mastertgsoftrst, uint8_t *regsoftrst, uint8_t *radiocntlsoftrst, uint8_t *swintreq, uint8_t *scanabort, uint8_t *rftestabort, uint8_t *pageinqabort, uint8_t *sniffabort, uint8_t *lmpflowdsb, uint8_t *cryptdsb, uint8_t *crcdsb, uint8_t *whitdsb, uint8_t *hopdsb, uint8_t *flowdsb, uint8_t *arqndsb, uint8_t *seqndsb, uint8_t *cxtxbsyena, uint8_t *cxrxbsyena, uint8_t *cxdnabort, uint8_t *rwbten, uint8_t *nwinsize)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);

    *mastersoftrst = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mastertgsoftrst = (localVal & ((uint32_t)0x40000000)) >> 30;
    *regsoftrst = (localVal & ((uint32_t)0x20000000)) >> 29;
    *radiocntlsoftrst = (localVal & ((uint32_t)0x10000000)) >> 28;
    *swintreq = (localVal & ((uint32_t)0x08000000)) >> 27;
    *scanabort = (localVal & ((uint32_t)0x00800000)) >> 23;
    *rftestabort = (localVal & ((uint32_t)0x00400000)) >> 22;
    *pageinqabort = (localVal & ((uint32_t)0x00200000)) >> 21;
    *sniffabort = (localVal & ((uint32_t)0x00100000)) >> 20;
    *lmpflowdsb = (localVal & ((uint32_t)0x00080000)) >> 19;
    *cryptdsb = (localVal & ((uint32_t)0x00040000)) >> 18;
    *crcdsb = (localVal & ((uint32_t)0x00020000)) >> 17;
    *whitdsb = (localVal & ((uint32_t)0x00010000)) >> 16;
    *hopdsb = (localVal & ((uint32_t)0x00008000)) >> 15;
    *flowdsb = (localVal & ((uint32_t)0x00004000)) >> 14;
    *arqndsb = (localVal & ((uint32_t)0x00002000)) >> 13;
    *seqndsb = (localVal & ((uint32_t)0x00001000)) >> 12;
    *cxtxbsyena = (localVal & ((uint32_t)0x00000800)) >> 11;
    *cxrxbsyena = (localVal & ((uint32_t)0x00000400)) >> 10;
    *cxdnabort = (localVal & ((uint32_t)0x00000200)) >> 9;
    *rwbten = (localVal & ((uint32_t)0x00000100)) >> 8;
    *nwinsize = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t bt_rwbtcntl_master_soft_rst_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_rwbtcntl_master_soft_rst_setf(uint8_t mastersoftrst)
{
    ASSERT_ERR((((uint32_t)mastersoftrst << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)mastersoftrst << 31));
}

__INLINE uint8_t bt_rwbtcntl_master_tgsoft_rst_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void bt_rwbtcntl_master_tgsoft_rst_setf(uint8_t mastertgsoftrst)
{
    ASSERT_ERR((((uint32_t)mastertgsoftrst << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)mastertgsoftrst << 30));
}

__INLINE uint8_t bt_rwbtcntl_reg_soft_rst_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void bt_rwbtcntl_reg_soft_rst_setf(uint8_t regsoftrst)
{
    ASSERT_ERR((((uint32_t)regsoftrst << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)regsoftrst << 29));
}

__INLINE uint8_t bt_rwbtcntl_radiocntl_soft_rst_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void bt_rwbtcntl_radiocntl_soft_rst_setf(uint8_t radiocntlsoftrst)
{
    ASSERT_ERR((((uint32_t)radiocntlsoftrst << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)radiocntlsoftrst << 28));
}

__INLINE uint8_t bt_rwbtcntl_swint_req_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void bt_rwbtcntl_swint_req_setf(uint8_t swintreq)
{
    ASSERT_ERR((((uint32_t)swintreq << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)swintreq << 27));
}

__INLINE uint8_t bt_rwbtcntl_scan_abort_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bt_rwbtcntl_scan_abort_setf(uint8_t scanabort)
{
    ASSERT_ERR((((uint32_t)scanabort << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)scanabort << 23));
}

__INLINE uint8_t bt_rwbtcntl_rftest_abort_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bt_rwbtcntl_rftest_abort_setf(uint8_t rftestabort)
{
    ASSERT_ERR((((uint32_t)rftestabort << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)rftestabort << 22));
}

__INLINE uint8_t bt_rwbtcntl_pageinq_abort_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void bt_rwbtcntl_pageinq_abort_setf(uint8_t pageinqabort)
{
    ASSERT_ERR((((uint32_t)pageinqabort << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)pageinqabort << 21));
}

__INLINE uint8_t bt_rwbtcntl_sniff_abort_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void bt_rwbtcntl_sniff_abort_setf(uint8_t sniffabort)
{
    ASSERT_ERR((((uint32_t)sniffabort << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)sniffabort << 20));
}

__INLINE uint8_t bt_rwbtcntl_lmpflowdsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void bt_rwbtcntl_lmpflowdsb_setf(uint8_t lmpflowdsb)
{
    ASSERT_ERR((((uint32_t)lmpflowdsb << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)lmpflowdsb << 19));
}

__INLINE uint8_t bt_rwbtcntl_cryptdsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void bt_rwbtcntl_cryptdsb_setf(uint8_t cryptdsb)
{
    ASSERT_ERR((((uint32_t)cryptdsb << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)cryptdsb << 18));
}

__INLINE uint8_t bt_rwbtcntl_crcdsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void bt_rwbtcntl_crcdsb_setf(uint8_t crcdsb)
{
    ASSERT_ERR((((uint32_t)crcdsb << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)crcdsb << 17));
}

__INLINE uint8_t bt_rwbtcntl_whitdsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bt_rwbtcntl_whitdsb_setf(uint8_t whitdsb)
{
    ASSERT_ERR((((uint32_t)whitdsb << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)whitdsb << 16));
}

__INLINE uint8_t bt_rwbtcntl_hopdsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_rwbtcntl_hopdsb_setf(uint8_t hopdsb)
{
    ASSERT_ERR((((uint32_t)hopdsb << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)hopdsb << 15));
}

__INLINE uint8_t bt_rwbtcntl_flowdsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_rwbtcntl_flowdsb_setf(uint8_t flowdsb)
{
    ASSERT_ERR((((uint32_t)flowdsb << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)flowdsb << 14));
}

__INLINE uint8_t bt_rwbtcntl_arqndsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_rwbtcntl_arqndsb_setf(uint8_t arqndsb)
{
    ASSERT_ERR((((uint32_t)arqndsb << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)arqndsb << 13));
}

__INLINE uint8_t bt_rwbtcntl_seqndsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_rwbtcntl_seqndsb_setf(uint8_t seqndsb)
{
    ASSERT_ERR((((uint32_t)seqndsb << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)seqndsb << 12));
}

__INLINE uint8_t bt_rwbtcntl_cx_txbsyena_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void bt_rwbtcntl_cx_txbsyena_setf(uint8_t cxtxbsyena)
{
    ASSERT_ERR((((uint32_t)cxtxbsyena << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)cxtxbsyena << 11));
}

__INLINE uint8_t bt_rwbtcntl_cx_rxbsyena_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bt_rwbtcntl_cx_rxbsyena_setf(uint8_t cxrxbsyena)
{
    ASSERT_ERR((((uint32_t)cxrxbsyena << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)cxrxbsyena << 10));
}

__INLINE uint8_t bt_rwbtcntl_cx_dnabort_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void bt_rwbtcntl_cx_dnabort_setf(uint8_t cxdnabort)
{
    ASSERT_ERR((((uint32_t)cxdnabort << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)cxdnabort << 9));
}

__INLINE uint8_t bt_rwbtcntl_rwbten_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bt_rwbtcntl_rwbten_setf(uint8_t rwbten)
{
    ASSERT_ERR((((uint32_t)rwbten << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rwbten << 8));
}

__INLINE uint8_t bt_rwbtcntl_nwinsize_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

__INLINE void bt_rwbtcntl_nwinsize_setf(uint8_t nwinsize)
{
    ASSERT_ERR((((uint32_t)nwinsize << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_BT_WR(BT_RWBTCNTL_ADDR, (REG_BT_RD(BT_RWBTCNTL_ADDR) & ~((uint32_t)0x0000003F)) | ((uint32_t)nwinsize << 0));
}

/**
 * @brief VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24                  TYP   0xB
 *  23:16                  REL   0x0
 *  15:08                  UPG   0x6
 *  07:00                BUILD   0x0
 * </pre>
 */
#define BT_VERSION_ADDR   0x50090404
#define BT_VERSION_OFFSET 0x00000004
#define BT_VERSION_INDEX  0x00000001
#define BT_VERSION_RESET  0x0B000600

__INLINE uint32_t bt_version_get(void)
{
    return REG_BT_RD(BT_VERSION_ADDR);
}

// field definitions
#define BT_TYP_MASK     ((uint32_t)0xFF000000)
#define BT_TYP_LSB      24
#define BT_TYP_WIDTH    ((uint32_t)0x00000008)
#define BT_REL_MASK     ((uint32_t)0x00FF0000)
#define BT_REL_LSB      16
#define BT_REL_WIDTH    ((uint32_t)0x00000008)
#define BT_UPG_MASK     ((uint32_t)0x0000FF00)
#define BT_UPG_LSB      8
#define BT_UPG_WIDTH    ((uint32_t)0x00000008)
#define BT_BUILD_MASK   ((uint32_t)0x000000FF)
#define BT_BUILD_LSB    0
#define BT_BUILD_WIDTH  ((uint32_t)0x00000008)

#define BT_TYP_RST      0xB
#define BT_REL_RST      0x0
#define BT_UPG_RST      0x6
#define BT_BUILD_RST    0x0

__INLINE void bt_version_unpack(uint8_t *typ, uint8_t *rel, uint8_t *upg, uint8_t *build)
{
    uint32_t localVal = REG_BT_RD(BT_VERSION_ADDR);

    *typ = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rel = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *upg = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *build = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bt_version_typ_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_VERSION_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t bt_version_rel_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t bt_version_upg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t bt_version_build_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief RWBTCONF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31               DMMODE   0
 *     30              MWSCOEX   1
 *     29             WLANCOEX   1
 *     28           CORRELATOR   0
 *     27              MWSWCI2   1
 *     26              MWSWCI1   1
 *  25:24             VXPORTNB   0x3
 *     23                  PCM   1
 *  22:16                 RFIF   0x2
 *     15               USEDBG   1
 *     14             DECIPHER   1
 *  13:08              CLK_SEL   0x8
 *     07              INTMODE   1
 *     06             BUS_TYPE   0
 *  04:00           ADDR_WIDTH   0x10
 * </pre>
 */
#define BT_RWBTCONF_ADDR   0x50090408
#define BT_RWBTCONF_OFFSET 0x00000008
#define BT_RWBTCONF_INDEX  0x00000002
#define BT_RWBTCONF_RESET  0x6F82C890

__INLINE uint32_t bt_rwbtconf_get(void)
{
    return REG_BT_RD(BT_RWBTCONF_ADDR);
}

// field definitions
#define BT_DMMODE_BIT        ((uint32_t)0x80000000)
#define BT_DMMODE_POS        31
#define BT_MWSCOEX_BIT       ((uint32_t)0x40000000)
#define BT_MWSCOEX_POS       30
#define BT_WLANCOEX_BIT      ((uint32_t)0x20000000)
#define BT_WLANCOEX_POS      29
#define BT_CORRELATOR_BIT    ((uint32_t)0x10000000)
#define BT_CORRELATOR_POS    28
#define BT_MWSWCI2_BIT       ((uint32_t)0x08000000)
#define BT_MWSWCI2_POS       27
#define BT_MWSWCI1_BIT       ((uint32_t)0x04000000)
#define BT_MWSWCI1_POS       26
#define BT_VXPORTNB_MASK     ((uint32_t)0x03000000)
#define BT_VXPORTNB_LSB      24
#define BT_VXPORTNB_WIDTH    ((uint32_t)0x00000002)
#define BT_PCM_BIT           ((uint32_t)0x00800000)
#define BT_PCM_POS           23
#define BT_RFIF_MASK         ((uint32_t)0x007F0000)
#define BT_RFIF_LSB          16
#define BT_RFIF_WIDTH        ((uint32_t)0x00000007)
#define BT_USEDBG_BIT        ((uint32_t)0x00008000)
#define BT_USEDBG_POS        15
#define BT_DECIPHER_BIT      ((uint32_t)0x00004000)
#define BT_DECIPHER_POS      14
#define BT_CLK_SEL_MASK      ((uint32_t)0x00003F00)
#define BT_CLK_SEL_LSB       8
#define BT_CLK_SEL_WIDTH     ((uint32_t)0x00000006)
#define BT_INTMODE_BIT       ((uint32_t)0x00000080)
#define BT_INTMODE_POS       7
#define BT_BUS_TYPE_BIT      ((uint32_t)0x00000040)
#define BT_BUS_TYPE_POS      6
#define BT_ADDR_WIDTH_MASK   ((uint32_t)0x0000001F)
#define BT_ADDR_WIDTH_LSB    0
#define BT_ADDR_WIDTH_WIDTH  ((uint32_t)0x00000005)

#define BT_DMMODE_RST        0x0
#define BT_MWSCOEX_RST       0x1
#define BT_WLANCOEX_RST      0x1
#define BT_CORRELATOR_RST    0x0
#define BT_MWSWCI2_RST       0x1
#define BT_MWSWCI1_RST       0x1
#define BT_VXPORTNB_RST      0x3
#define BT_PCM_RST           0x1
#define BT_RFIF_RST          0x2
#define BT_USEDBG_RST        0x1
#define BT_DECIPHER_RST      0x1
#define BT_CLK_SEL_RST       0x8
#define BT_INTMODE_RST       0x1
#define BT_BUS_TYPE_RST      0x0
#define BT_ADDR_WIDTH_RST    0x10

__INLINE void bt_rwbtconf_unpack(uint8_t *dmmode, uint8_t *mwscoex, uint8_t *wlancoex, uint8_t *correlator, uint8_t *mwswci2, uint8_t *mwswci1, uint8_t *vxportnb, uint8_t *pcm, uint8_t *rfif, uint8_t *usedbg, uint8_t *decipher, uint8_t *clksel, uint8_t *intmode, uint8_t *bustype, uint8_t *addrwidth)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);

    *dmmode = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mwscoex = (localVal & ((uint32_t)0x40000000)) >> 30;
    *wlancoex = (localVal & ((uint32_t)0x20000000)) >> 29;
    *correlator = (localVal & ((uint32_t)0x10000000)) >> 28;
    *mwswci2 = (localVal & ((uint32_t)0x08000000)) >> 27;
    *mwswci1 = (localVal & ((uint32_t)0x04000000)) >> 26;
    *vxportnb = (localVal & ((uint32_t)0x03000000)) >> 24;
    *pcm = (localVal & ((uint32_t)0x00800000)) >> 23;
    *rfif = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *usedbg = (localVal & ((uint32_t)0x00008000)) >> 15;
    *decipher = (localVal & ((uint32_t)0x00004000)) >> 14;
    *clksel = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *intmode = (localVal & ((uint32_t)0x00000080)) >> 7;
    *bustype = (localVal & ((uint32_t)0x00000040)) >> 6;
    *addrwidth = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t bt_rwbtconf_dmmode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t bt_rwbtconf_mwscoex_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE uint8_t bt_rwbtconf_wlancoex_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE uint8_t bt_rwbtconf_correlator_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE uint8_t bt_rwbtconf_mwswci2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE uint8_t bt_rwbtconf_mwswci1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE uint8_t bt_rwbtconf_vxportnb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE uint8_t bt_rwbtconf_pcm_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE uint8_t bt_rwbtconf_rfif_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE uint8_t bt_rwbtconf_usedbg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t bt_rwbtconf_decipher_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t bt_rwbtconf_clk_sel_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE uint8_t bt_rwbtconf_intmode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t bt_rwbtconf_bus_type_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t bt_rwbtconf_addr_width_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RWBTCONF_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

/**
 * @brief INTCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16          ERRORINTMSK   0
 *     15         AUDIOINT2MSK   0
 *     14         AUDIOINT1MSK   0
 *     13         AUDIOINT0MSK   0
 *     12       MWSWCIRXINTMSK   0
 *     11       MWSWCITXINTMSK   0
 *     10         MTOFFINT1MSK   0
 *     09         MTOFFINT0MSK   0
 *     08         FRSYNCINTMSK   0
 *     04             RXINTMSK   0
 *     02        SKIPFRMINTMSK   0
 *     01         ENDFRMINTMSK   1
 *     00       STARTFRMINTMSK   1
 * </pre>
 */
#define BT_INTCNTL0_ADDR   0x5009040C
#define BT_INTCNTL0_OFFSET 0x0000000C
#define BT_INTCNTL0_INDEX  0x00000003
#define BT_INTCNTL0_RESET  0x00000003

__INLINE uint32_t bt_intcntl0_get(void)
{
    return REG_BT_RD(BT_INTCNTL0_ADDR);
}

__INLINE void bt_intcntl0_set(uint32_t value)
{
    REG_BT_WR(BT_INTCNTL0_ADDR, value);
}

// field definitions
#define BT_ERRORINTMSK_BIT       ((uint32_t)0x00010000)
#define BT_ERRORINTMSK_POS       16
#define BT_AUDIOINT2MSK_BIT      ((uint32_t)0x00008000)
#define BT_AUDIOINT2MSK_POS      15
#define BT_AUDIOINT1MSK_BIT      ((uint32_t)0x00004000)
#define BT_AUDIOINT1MSK_POS      14
#define BT_AUDIOINT0MSK_BIT      ((uint32_t)0x00002000)
#define BT_AUDIOINT0MSK_POS      13
#define BT_MWSWCIRXINTMSK_BIT    ((uint32_t)0x00001000)
#define BT_MWSWCIRXINTMSK_POS    12
#define BT_MWSWCITXINTMSK_BIT    ((uint32_t)0x00000800)
#define BT_MWSWCITXINTMSK_POS    11
#define BT_MTOFFINT1MSK_BIT      ((uint32_t)0x00000400)
#define BT_MTOFFINT1MSK_POS      10
#define BT_MTOFFINT0MSK_BIT      ((uint32_t)0x00000200)
#define BT_MTOFFINT0MSK_POS      9
#define BT_FRSYNCINTMSK_BIT      ((uint32_t)0x00000100)
#define BT_FRSYNCINTMSK_POS      8
#define BT_RXINTMSK_BIT          ((uint32_t)0x00000010)
#define BT_RXINTMSK_POS          4
#define BT_SKIPFRMINTMSK_BIT     ((uint32_t)0x00000004)
#define BT_SKIPFRMINTMSK_POS     2
#define BT_ENDFRMINTMSK_BIT      ((uint32_t)0x00000002)
#define BT_ENDFRMINTMSK_POS      1
#define BT_STARTFRMINTMSK_BIT    ((uint32_t)0x00000001)
#define BT_STARTFRMINTMSK_POS    0

#define BT_ERRORINTMSK_RST       0x0
#define BT_AUDIOINT2MSK_RST      0x0
#define BT_AUDIOINT1MSK_RST      0x0
#define BT_AUDIOINT0MSK_RST      0x0
#define BT_MWSWCIRXINTMSK_RST    0x0
#define BT_MWSWCITXINTMSK_RST    0x0
#define BT_MTOFFINT1MSK_RST      0x0
#define BT_MTOFFINT0MSK_RST      0x0
#define BT_FRSYNCINTMSK_RST      0x0
#define BT_RXINTMSK_RST          0x0
#define BT_SKIPFRMINTMSK_RST     0x0
#define BT_ENDFRMINTMSK_RST      0x1
#define BT_STARTFRMINTMSK_RST    0x1

__INLINE void bt_intcntl0_pack(uint8_t errorintmsk, uint8_t audioint2msk, uint8_t audioint1msk, uint8_t audioint0msk, uint8_t mwswcirxintmsk, uint8_t mwswcitxintmsk, uint8_t mtoffint1msk, uint8_t mtoffint0msk, uint8_t frsyncintmsk, uint8_t rxintmsk, uint8_t skipfrmintmsk, uint8_t endfrmintmsk, uint8_t startfrmintmsk)
{
    ASSERT_ERR((((uint32_t)errorintmsk << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)audioint2msk << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)audioint1msk << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)audioint0msk << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)mwswcirxintmsk << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)mwswcitxintmsk << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)mtoffint1msk << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)mtoffint0msk << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)frsyncintmsk << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rxintmsk << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)skipfrmintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)endfrmintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)startfrmintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, ((uint32_t)errorintmsk << 16) | ((uint32_t)audioint2msk << 15) | ((uint32_t)audioint1msk << 14) | ((uint32_t)audioint0msk << 13) | ((uint32_t)mwswcirxintmsk << 12) | ((uint32_t)mwswcitxintmsk << 11) | ((uint32_t)mtoffint1msk << 10) | ((uint32_t)mtoffint0msk << 9) | ((uint32_t)frsyncintmsk << 8) | ((uint32_t)rxintmsk << 4) | ((uint32_t)skipfrmintmsk << 2) | ((uint32_t)endfrmintmsk << 1) | ((uint32_t)startfrmintmsk << 0));
}

__INLINE void bt_intcntl0_unpack(uint8_t *errorintmsk, uint8_t *audioint2msk, uint8_t *audioint1msk, uint8_t *audioint0msk, uint8_t *mwswcirxintmsk, uint8_t *mwswcitxintmsk, uint8_t *mtoffint1msk, uint8_t *mtoffint0msk, uint8_t *frsyncintmsk, uint8_t *rxintmsk, uint8_t *skipfrmintmsk, uint8_t *endfrmintmsk, uint8_t *startfrmintmsk)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);

    *errorintmsk = (localVal & ((uint32_t)0x00010000)) >> 16;
    *audioint2msk = (localVal & ((uint32_t)0x00008000)) >> 15;
    *audioint1msk = (localVal & ((uint32_t)0x00004000)) >> 14;
    *audioint0msk = (localVal & ((uint32_t)0x00002000)) >> 13;
    *mwswcirxintmsk = (localVal & ((uint32_t)0x00001000)) >> 12;
    *mwswcitxintmsk = (localVal & ((uint32_t)0x00000800)) >> 11;
    *mtoffint1msk = (localVal & ((uint32_t)0x00000400)) >> 10;
    *mtoffint0msk = (localVal & ((uint32_t)0x00000200)) >> 9;
    *frsyncintmsk = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxintmsk = (localVal & ((uint32_t)0x00000010)) >> 4;
    *skipfrmintmsk = (localVal & ((uint32_t)0x00000004)) >> 2;
    *endfrmintmsk = (localVal & ((uint32_t)0x00000002)) >> 1;
    *startfrmintmsk = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_intcntl0_errorintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bt_intcntl0_errorintmsk_setf(uint8_t errorintmsk)
{
    ASSERT_ERR((((uint32_t)errorintmsk << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)errorintmsk << 16));
}

__INLINE uint8_t bt_intcntl0_audioint2msk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_intcntl0_audioint2msk_setf(uint8_t audioint2msk)
{
    ASSERT_ERR((((uint32_t)audioint2msk << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)audioint2msk << 15));
}

__INLINE uint8_t bt_intcntl0_audioint1msk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_intcntl0_audioint1msk_setf(uint8_t audioint1msk)
{
    ASSERT_ERR((((uint32_t)audioint1msk << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)audioint1msk << 14));
}

__INLINE uint8_t bt_intcntl0_audioint0msk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_intcntl0_audioint0msk_setf(uint8_t audioint0msk)
{
    ASSERT_ERR((((uint32_t)audioint0msk << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)audioint0msk << 13));
}

__INLINE uint8_t bt_intcntl0_mwswcirxintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_intcntl0_mwswcirxintmsk_setf(uint8_t mwswcirxintmsk)
{
    ASSERT_ERR((((uint32_t)mwswcirxintmsk << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)mwswcirxintmsk << 12));
}

__INLINE uint8_t bt_intcntl0_mwswcitxintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void bt_intcntl0_mwswcitxintmsk_setf(uint8_t mwswcitxintmsk)
{
    ASSERT_ERR((((uint32_t)mwswcitxintmsk << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)mwswcitxintmsk << 11));
}

__INLINE uint8_t bt_intcntl0_mtoffint1msk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bt_intcntl0_mtoffint1msk_setf(uint8_t mtoffint1msk)
{
    ASSERT_ERR((((uint32_t)mtoffint1msk << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)mtoffint1msk << 10));
}

__INLINE uint8_t bt_intcntl0_mtoffint0msk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void bt_intcntl0_mtoffint0msk_setf(uint8_t mtoffint0msk)
{
    ASSERT_ERR((((uint32_t)mtoffint0msk << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)mtoffint0msk << 9));
}

__INLINE uint8_t bt_intcntl0_frsyncintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bt_intcntl0_frsyncintmsk_setf(uint8_t frsyncintmsk)
{
    ASSERT_ERR((((uint32_t)frsyncintmsk << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)frsyncintmsk << 8));
}

__INLINE uint8_t bt_intcntl0_rxintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_intcntl0_rxintmsk_setf(uint8_t rxintmsk)
{
    ASSERT_ERR((((uint32_t)rxintmsk << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxintmsk << 4));
}

__INLINE uint8_t bt_intcntl0_skipfrmintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_intcntl0_skipfrmintmsk_setf(uint8_t skipfrmintmsk)
{
    ASSERT_ERR((((uint32_t)skipfrmintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)skipfrmintmsk << 2));
}

__INLINE uint8_t bt_intcntl0_endfrmintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_intcntl0_endfrmintmsk_setf(uint8_t endfrmintmsk)
{
    ASSERT_ERR((((uint32_t)endfrmintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)endfrmintmsk << 1));
}

__INLINE uint8_t bt_intcntl0_startfrmintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_intcntl0_startfrmintmsk_setf(uint8_t startfrmintmsk)
{
    ASSERT_ERR((((uint32_t)startfrmintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_INTCNTL0_ADDR, (REG_BT_RD(BT_INTCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)startfrmintmsk << 0));
}

/**
 * @brief INTSTAT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16         ERRORINTSTAT   0
 *     15        AUDIOINT2STAT   0
 *     14        AUDIOINT1STAT   0
 *     13        AUDIOINT0STAT   0
 *     12      MWSWCIRXINTSTAT   0
 *     11      MWSWCITXINTSTAT   0
 *     10        MTOFFINT1STAT   0
 *     09        MTOFFINT0STAT   0
 *     08        FRSYNCINTSTAT   0
 * </pre>
 */
#define BT_INTSTAT0_ADDR   0x50090410
#define BT_INTSTAT0_OFFSET 0x00000010
#define BT_INTSTAT0_INDEX  0x00000004
#define BT_INTSTAT0_RESET  0x00000000

__INLINE uint32_t bt_intstat0_get(void)
{
    return REG_BT_RD(BT_INTSTAT0_ADDR);
}

// field definitions
#define BT_ERRORINTSTAT_BIT       ((uint32_t)0x00010000)
#define BT_ERRORINTSTAT_POS       16
#define BT_AUDIOINT2STAT_BIT      ((uint32_t)0x00008000)
#define BT_AUDIOINT2STAT_POS      15
#define BT_AUDIOINT1STAT_BIT      ((uint32_t)0x00004000)
#define BT_AUDIOINT1STAT_POS      14
#define BT_AUDIOINT0STAT_BIT      ((uint32_t)0x00002000)
#define BT_AUDIOINT0STAT_POS      13
#define BT_MWSWCIRXINTSTAT_BIT    ((uint32_t)0x00001000)
#define BT_MWSWCIRXINTSTAT_POS    12
#define BT_MWSWCITXINTSTAT_BIT    ((uint32_t)0x00000800)
#define BT_MWSWCITXINTSTAT_POS    11
#define BT_MTOFFINT1STAT_BIT      ((uint32_t)0x00000400)
#define BT_MTOFFINT1STAT_POS      10
#define BT_MTOFFINT0STAT_BIT      ((uint32_t)0x00000200)
#define BT_MTOFFINT0STAT_POS      9
#define BT_FRSYNCINTSTAT_BIT      ((uint32_t)0x00000100)
#define BT_FRSYNCINTSTAT_POS      8

#define BT_ERRORINTSTAT_RST       0x0
#define BT_AUDIOINT2STAT_RST      0x0
#define BT_AUDIOINT1STAT_RST      0x0
#define BT_AUDIOINT0STAT_RST      0x0
#define BT_MWSWCIRXINTSTAT_RST    0x0
#define BT_MWSWCITXINTSTAT_RST    0x0
#define BT_MTOFFINT1STAT_RST      0x0
#define BT_MTOFFINT0STAT_RST      0x0
#define BT_FRSYNCINTSTAT_RST      0x0

__INLINE void bt_intstat0_unpack(uint8_t *errorintstat, uint8_t *audioint2stat, uint8_t *audioint1stat, uint8_t *audioint0stat, uint8_t *mwswcirxintstat, uint8_t *mwswcitxintstat, uint8_t *mtoffint1stat, uint8_t *mtoffint0stat, uint8_t *frsyncintstat)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);

    *errorintstat = (localVal & ((uint32_t)0x00010000)) >> 16;
    *audioint2stat = (localVal & ((uint32_t)0x00008000)) >> 15;
    *audioint1stat = (localVal & ((uint32_t)0x00004000)) >> 14;
    *audioint0stat = (localVal & ((uint32_t)0x00002000)) >> 13;
    *mwswcirxintstat = (localVal & ((uint32_t)0x00001000)) >> 12;
    *mwswcitxintstat = (localVal & ((uint32_t)0x00000800)) >> 11;
    *mtoffint1stat = (localVal & ((uint32_t)0x00000400)) >> 10;
    *mtoffint0stat = (localVal & ((uint32_t)0x00000200)) >> 9;
    *frsyncintstat = (localVal & ((uint32_t)0x00000100)) >> 8;
}

__INLINE uint8_t bt_intstat0_errorintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint8_t bt_intstat0_audioint2stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t bt_intstat0_audioint1stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t bt_intstat0_audioint0stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t bt_intstat0_mwswcirxintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t bt_intstat0_mwswcitxintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t bt_intstat0_mtoffint1stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t bt_intstat0_mtoffint0stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t bt_intstat0_frsyncintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

/**
 * @brief INTACK0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16          ERRORINTACK   0
 *     15         AUDIOINT2ACK   0
 *     14         AUDIOINT1ACK   0
 *     13         AUDIOINT0ACK   0
 *     12       MWSWCIRXINTACK   0
 *     11       MWSWCITXINTACK   0
 *     10         MTOFFINT1ACK   0
 *     09         MTOFFINT0ACK   0
 *     08         FRSYNCINTACK   0
 * </pre>
 */
#define BT_INTACK0_ADDR   0x50090414
#define BT_INTACK0_OFFSET 0x00000014
#define BT_INTACK0_INDEX  0x00000005
#define BT_INTACK0_RESET  0x00000000

__INLINE uint32_t bt_intack0_get(void)
{
    return REG_BT_RD(BT_INTACK0_ADDR);
}

__INLINE void bt_intack0_clear(uint32_t value)
{
    REG_BT_WR(BT_INTACK0_ADDR, value);
}

// field definitions
#define BT_ERRORINTACK_BIT       ((uint32_t)0x00010000)
#define BT_ERRORINTACK_POS       16
#define BT_AUDIOINT2ACK_BIT      ((uint32_t)0x00008000)
#define BT_AUDIOINT2ACK_POS      15
#define BT_AUDIOINT1ACK_BIT      ((uint32_t)0x00004000)
#define BT_AUDIOINT1ACK_POS      14
#define BT_AUDIOINT0ACK_BIT      ((uint32_t)0x00002000)
#define BT_AUDIOINT0ACK_POS      13
#define BT_MWSWCIRXINTACK_BIT    ((uint32_t)0x00001000)
#define BT_MWSWCIRXINTACK_POS    12
#define BT_MWSWCITXINTACK_BIT    ((uint32_t)0x00000800)
#define BT_MWSWCITXINTACK_POS    11
#define BT_MTOFFINT1ACK_BIT      ((uint32_t)0x00000400)
#define BT_MTOFFINT1ACK_POS      10
#define BT_MTOFFINT0ACK_BIT      ((uint32_t)0x00000200)
#define BT_MTOFFINT0ACK_POS      9
#define BT_FRSYNCINTACK_BIT      ((uint32_t)0x00000100)
#define BT_FRSYNCINTACK_POS      8

#define BT_ERRORINTACK_RST       0x0
#define BT_AUDIOINT2ACK_RST      0x0
#define BT_AUDIOINT1ACK_RST      0x0
#define BT_AUDIOINT0ACK_RST      0x0
#define BT_MWSWCIRXINTACK_RST    0x0
#define BT_MWSWCITXINTACK_RST    0x0
#define BT_MTOFFINT1ACK_RST      0x0
#define BT_MTOFFINT0ACK_RST      0x0
#define BT_FRSYNCINTACK_RST      0x0

__INLINE void bt_intack0_pack(uint8_t errorintack, uint8_t audioint2ack, uint8_t audioint1ack, uint8_t audioint0ack, uint8_t mwswcirxintack, uint8_t mwswcitxintack, uint8_t mtoffint1ack, uint8_t mtoffint0ack, uint8_t frsyncintack)
{
    ASSERT_ERR((((uint32_t)errorintack << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)audioint2ack << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)audioint1ack << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)audioint0ack << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)mwswcirxintack << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)mwswcitxintack << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)mtoffint1ack << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)mtoffint0ack << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)frsyncintack << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, ((uint32_t)errorintack << 16) | ((uint32_t)audioint2ack << 15) | ((uint32_t)audioint1ack << 14) | ((uint32_t)audioint0ack << 13) | ((uint32_t)mwswcirxintack << 12) | ((uint32_t)mwswcitxintack << 11) | ((uint32_t)mtoffint1ack << 10) | ((uint32_t)mtoffint0ack << 9) | ((uint32_t)frsyncintack << 8));
}

__INLINE void bt_intack0_unpack(uint8_t *errorintack, uint8_t *audioint2ack, uint8_t *audioint1ack, uint8_t *audioint0ack, uint8_t *mwswcirxintack, uint8_t *mwswcitxintack, uint8_t *mtoffint1ack, uint8_t *mtoffint0ack, uint8_t *frsyncintack)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);

    *errorintack = (localVal & ((uint32_t)0x00010000)) >> 16;
    *audioint2ack = (localVal & ((uint32_t)0x00008000)) >> 15;
    *audioint1ack = (localVal & ((uint32_t)0x00004000)) >> 14;
    *audioint0ack = (localVal & ((uint32_t)0x00002000)) >> 13;
    *mwswcirxintack = (localVal & ((uint32_t)0x00001000)) >> 12;
    *mwswcitxintack = (localVal & ((uint32_t)0x00000800)) >> 11;
    *mtoffint1ack = (localVal & ((uint32_t)0x00000400)) >> 10;
    *mtoffint0ack = (localVal & ((uint32_t)0x00000200)) >> 9;
    *frsyncintack = (localVal & ((uint32_t)0x00000100)) >> 8;
}

__INLINE uint8_t bt_intack0_errorintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bt_intack0_errorintack_clearf(uint8_t errorintack)
{
    ASSERT_ERR((((uint32_t)errorintack << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)errorintack << 16);
}

__INLINE uint8_t bt_intack0_audioint2ack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_intack0_audioint2ack_clearf(uint8_t audioint2ack)
{
    ASSERT_ERR((((uint32_t)audioint2ack << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)audioint2ack << 15);
}

__INLINE uint8_t bt_intack0_audioint1ack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_intack0_audioint1ack_clearf(uint8_t audioint1ack)
{
    ASSERT_ERR((((uint32_t)audioint1ack << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)audioint1ack << 14);
}

__INLINE uint8_t bt_intack0_audioint0ack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_intack0_audioint0ack_clearf(uint8_t audioint0ack)
{
    ASSERT_ERR((((uint32_t)audioint0ack << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)audioint0ack << 13);
}

__INLINE uint8_t bt_intack0_mwswcirxintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_intack0_mwswcirxintack_clearf(uint8_t mwswcirxintack)
{
    ASSERT_ERR((((uint32_t)mwswcirxintack << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)mwswcirxintack << 12);
}

__INLINE uint8_t bt_intack0_mwswcitxintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void bt_intack0_mwswcitxintack_clearf(uint8_t mwswcitxintack)
{
    ASSERT_ERR((((uint32_t)mwswcitxintack << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)mwswcitxintack << 11);
}

__INLINE uint8_t bt_intack0_mtoffint1ack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bt_intack0_mtoffint1ack_clearf(uint8_t mtoffint1ack)
{
    ASSERT_ERR((((uint32_t)mtoffint1ack << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)mtoffint1ack << 10);
}

__INLINE uint8_t bt_intack0_mtoffint0ack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void bt_intack0_mtoffint0ack_clearf(uint8_t mtoffint0ack)
{
    ASSERT_ERR((((uint32_t)mtoffint0ack << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)mtoffint0ack << 9);
}

__INLINE uint8_t bt_intack0_frsyncintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bt_intack0_frsyncintack_clearf(uint8_t frsyncintack)
{
    ASSERT_ERR((((uint32_t)frsyncintack << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(BT_INTACK0_ADDR, (uint32_t)frsyncintack << 8);
}

/**
 * @brief INTCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:28         CLKNINTSRMSK   0x0
 *  27:24         CLKNINTSRVAL   0x0
 *     15           FIFOINTMSK   1
 *     07   TIMESTAMPTGT3INTMSK   0
 *     06   TIMESTAMPTGT2INTMSK   0
 *     05   TIMESTAMPTGT1INTMSK   0
 *     04        FINETGTINTMSK   0
 *     03             SWINTMSK   0
 *     02          CRYPTINTMSK   0
 *     01            SLPINTMSK   1
 *     00           CLKNINTMSK   1
 * </pre>
 */
#define BT_INTCNTL1_ADDR   0x50090418
#define BT_INTCNTL1_OFFSET 0x00000018
#define BT_INTCNTL1_INDEX  0x00000006
#define BT_INTCNTL1_RESET  0x00008003

__INLINE uint32_t bt_intcntl1_get(void)
{
    return REG_BT_RD(BT_INTCNTL1_ADDR);
}

__INLINE void bt_intcntl1_set(uint32_t value)
{
    REG_BT_WR(BT_INTCNTL1_ADDR, value);
}

// field definitions
#define BT_CLKNINTSRMSK_MASK          ((uint32_t)0x70000000)
#define BT_CLKNINTSRMSK_LSB           28
#define BT_CLKNINTSRMSK_WIDTH         ((uint32_t)0x00000003)
#define BT_CLKNINTSRVAL_MASK          ((uint32_t)0x0F000000)
#define BT_CLKNINTSRVAL_LSB           24
#define BT_CLKNINTSRVAL_WIDTH         ((uint32_t)0x00000004)
#define BT_FIFOINTMSK_BIT             ((uint32_t)0x00008000)
#define BT_FIFOINTMSK_POS             15
#define BT_TIMESTAMPTGT3INTMSK_BIT    ((uint32_t)0x00000080)
#define BT_TIMESTAMPTGT3INTMSK_POS    7
#define BT_TIMESTAMPTGT2INTMSK_BIT    ((uint32_t)0x00000040)
#define BT_TIMESTAMPTGT2INTMSK_POS    6
#define BT_TIMESTAMPTGT1INTMSK_BIT    ((uint32_t)0x00000020)
#define BT_TIMESTAMPTGT1INTMSK_POS    5
#define BT_FINETGTINTMSK_BIT          ((uint32_t)0x00000010)
#define BT_FINETGTINTMSK_POS          4
#define BT_SWINTMSK_BIT               ((uint32_t)0x00000008)
#define BT_SWINTMSK_POS               3
#define BT_CRYPTINTMSK_BIT            ((uint32_t)0x00000004)
#define BT_CRYPTINTMSK_POS            2
#define BT_SLPINTMSK_BIT              ((uint32_t)0x00000002)
#define BT_SLPINTMSK_POS              1
#define BT_CLKNINTMSK_BIT             ((uint32_t)0x00000001)
#define BT_CLKNINTMSK_POS             0

#define BT_CLKNINTSRMSK_RST           0x0
#define BT_CLKNINTSRVAL_RST           0x0
#define BT_FIFOINTMSK_RST             0x1
#define BT_TIMESTAMPTGT3INTMSK_RST    0x0
#define BT_TIMESTAMPTGT2INTMSK_RST    0x0
#define BT_TIMESTAMPTGT1INTMSK_RST    0x0
#define BT_FINETGTINTMSK_RST          0x0
#define BT_SWINTMSK_RST               0x0
#define BT_CRYPTINTMSK_RST            0x0
#define BT_SLPINTMSK_RST              0x1
#define BT_CLKNINTMSK_RST             0x1

__INLINE void bt_intcntl1_pack(uint8_t clknintsrmsk, uint8_t clknintsrval, uint8_t fifointmsk, uint8_t timestamptgt3intmsk, uint8_t timestamptgt2intmsk, uint8_t timestamptgt1intmsk, uint8_t finetgtintmsk, uint8_t swintmsk, uint8_t cryptintmsk, uint8_t slpintmsk, uint8_t clknintmsk)
{
    ASSERT_ERR((((uint32_t)clknintsrmsk << 28) & ~((uint32_t)0x70000000)) == 0);
    ASSERT_ERR((((uint32_t)clknintsrval << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)fifointmsk << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgt3intmsk << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgt2intmsk << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgt1intmsk << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)finetgtintmsk << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)swintmsk << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)cryptintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)slpintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)clknintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, ((uint32_t)clknintsrmsk << 28) | ((uint32_t)clknintsrval << 24) | ((uint32_t)fifointmsk << 15) | ((uint32_t)timestamptgt3intmsk << 7) | ((uint32_t)timestamptgt2intmsk << 6) | ((uint32_t)timestamptgt1intmsk << 5) | ((uint32_t)finetgtintmsk << 4) | ((uint32_t)swintmsk << 3) | ((uint32_t)cryptintmsk << 2) | ((uint32_t)slpintmsk << 1) | ((uint32_t)clknintmsk << 0));
}

__INLINE void bt_intcntl1_unpack(uint8_t *clknintsrmsk, uint8_t *clknintsrval, uint8_t *fifointmsk, uint8_t *timestamptgt3intmsk, uint8_t *timestamptgt2intmsk, uint8_t *timestamptgt1intmsk, uint8_t *finetgtintmsk, uint8_t *swintmsk, uint8_t *cryptintmsk, uint8_t *slpintmsk, uint8_t *clknintmsk)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);

    *clknintsrmsk = (localVal & ((uint32_t)0x70000000)) >> 28;
    *clknintsrval = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *fifointmsk = (localVal & ((uint32_t)0x00008000)) >> 15;
    *timestamptgt3intmsk = (localVal & ((uint32_t)0x00000080)) >> 7;
    *timestamptgt2intmsk = (localVal & ((uint32_t)0x00000040)) >> 6;
    *timestamptgt1intmsk = (localVal & ((uint32_t)0x00000020)) >> 5;
    *finetgtintmsk = (localVal & ((uint32_t)0x00000010)) >> 4;
    *swintmsk = (localVal & ((uint32_t)0x00000008)) >> 3;
    *cryptintmsk = (localVal & ((uint32_t)0x00000004)) >> 2;
    *slpintmsk = (localVal & ((uint32_t)0x00000002)) >> 1;
    *clknintmsk = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_intcntl1_clknintsrmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x70000000)) >> 28);
}

__INLINE void bt_intcntl1_clknintsrmsk_setf(uint8_t clknintsrmsk)
{
    ASSERT_ERR((((uint32_t)clknintsrmsk << 28) & ~((uint32_t)0x70000000)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x70000000)) | ((uint32_t)clknintsrmsk << 28));
}

__INLINE uint8_t bt_intcntl1_clknintsrval_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void bt_intcntl1_clknintsrval_setf(uint8_t clknintsrval)
{
    ASSERT_ERR((((uint32_t)clknintsrval << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)clknintsrval << 24));
}

__INLINE uint8_t bt_intcntl1_fifointmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_intcntl1_fifointmsk_setf(uint8_t fifointmsk)
{
    ASSERT_ERR((((uint32_t)fifointmsk << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)fifointmsk << 15));
}

__INLINE uint8_t bt_intcntl1_timestamptgt3intmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_intcntl1_timestamptgt3intmsk_setf(uint8_t timestamptgt3intmsk)
{
    ASSERT_ERR((((uint32_t)timestamptgt3intmsk << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)timestamptgt3intmsk << 7));
}

__INLINE uint8_t bt_intcntl1_timestamptgt2intmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bt_intcntl1_timestamptgt2intmsk_setf(uint8_t timestamptgt2intmsk)
{
    ASSERT_ERR((((uint32_t)timestamptgt2intmsk << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)timestamptgt2intmsk << 6));
}

__INLINE uint8_t bt_intcntl1_timestamptgt1intmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_intcntl1_timestamptgt1intmsk_setf(uint8_t timestamptgt1intmsk)
{
    ASSERT_ERR((((uint32_t)timestamptgt1intmsk << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)timestamptgt1intmsk << 5));
}

__INLINE uint8_t bt_intcntl1_finetgtintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_intcntl1_finetgtintmsk_setf(uint8_t finetgtintmsk)
{
    ASSERT_ERR((((uint32_t)finetgtintmsk << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)finetgtintmsk << 4));
}

__INLINE uint8_t bt_intcntl1_swintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_intcntl1_swintmsk_setf(uint8_t swintmsk)
{
    ASSERT_ERR((((uint32_t)swintmsk << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)swintmsk << 3));
}

__INLINE uint8_t bt_intcntl1_cryptintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_intcntl1_cryptintmsk_setf(uint8_t cryptintmsk)
{
    ASSERT_ERR((((uint32_t)cryptintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)cryptintmsk << 2));
}

__INLINE uint8_t bt_intcntl1_slpintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_intcntl1_slpintmsk_setf(uint8_t slpintmsk)
{
    ASSERT_ERR((((uint32_t)slpintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)slpintmsk << 1));
}

__INLINE uint8_t bt_intcntl1_clknintmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_intcntl1_clknintmsk_setf(uint8_t clknintmsk)
{
    ASSERT_ERR((((uint32_t)clknintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_INTCNTL1_ADDR, (REG_BT_RD(BT_INTCNTL1_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)clknintmsk << 0));
}

/**
 * @brief INTSTAT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15          FIFOINTSTAT   0
 *     07   TIMESTAMPTGT3INTSTAT   0
 *     06   TIMESTAMPTGT2INTSTAT   0
 *     05   TIMESTAMPTGT1INTSTAT   0
 *     04       FINETGTINTSTAT   0
 *     03            SWINTSTAT   0
 *     02         CRYPTINTSTAT   0
 *     01           SLPINTSTAT   0
 *     00          CLKNINTSTAT   0
 * </pre>
 */
#define BT_INTSTAT1_ADDR   0x5009041C
#define BT_INTSTAT1_OFFSET 0x0000001C
#define BT_INTSTAT1_INDEX  0x00000007
#define BT_INTSTAT1_RESET  0x00000000

__INLINE uint32_t bt_intstat1_get(void)
{
    return REG_BT_RD(BT_INTSTAT1_ADDR);
}

// field definitions
#define BT_FIFOINTSTAT_BIT             ((uint32_t)0x00008000)
#define BT_FIFOINTSTAT_POS             15
#define BT_TIMESTAMPTGT3INTSTAT_BIT    ((uint32_t)0x00000080)
#define BT_TIMESTAMPTGT3INTSTAT_POS    7
#define BT_TIMESTAMPTGT2INTSTAT_BIT    ((uint32_t)0x00000040)
#define BT_TIMESTAMPTGT2INTSTAT_POS    6
#define BT_TIMESTAMPTGT1INTSTAT_BIT    ((uint32_t)0x00000020)
#define BT_TIMESTAMPTGT1INTSTAT_POS    5
#define BT_FINETGTINTSTAT_BIT          ((uint32_t)0x00000010)
#define BT_FINETGTINTSTAT_POS          4
#define BT_SWINTSTAT_BIT               ((uint32_t)0x00000008)
#define BT_SWINTSTAT_POS               3
#define BT_CRYPTINTSTAT_BIT            ((uint32_t)0x00000004)
#define BT_CRYPTINTSTAT_POS            2
#define BT_SLPINTSTAT_BIT              ((uint32_t)0x00000002)
#define BT_SLPINTSTAT_POS              1
#define BT_CLKNINTSTAT_BIT             ((uint32_t)0x00000001)
#define BT_CLKNINTSTAT_POS             0

#define BT_FIFOINTSTAT_RST             0x0
#define BT_TIMESTAMPTGT3INTSTAT_RST    0x0
#define BT_TIMESTAMPTGT2INTSTAT_RST    0x0
#define BT_TIMESTAMPTGT1INTSTAT_RST    0x0
#define BT_FINETGTINTSTAT_RST          0x0
#define BT_SWINTSTAT_RST               0x0
#define BT_CRYPTINTSTAT_RST            0x0
#define BT_SLPINTSTAT_RST              0x0
#define BT_CLKNINTSTAT_RST             0x0

__INLINE void bt_intstat1_unpack(uint8_t *fifointstat, uint8_t *timestamptgt3intstat, uint8_t *timestamptgt2intstat, uint8_t *timestamptgt1intstat, uint8_t *finetgtintstat, uint8_t *swintstat, uint8_t *cryptintstat, uint8_t *slpintstat, uint8_t *clknintstat)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);

    *fifointstat = (localVal & ((uint32_t)0x00008000)) >> 15;
    *timestamptgt3intstat = (localVal & ((uint32_t)0x00000080)) >> 7;
    *timestamptgt2intstat = (localVal & ((uint32_t)0x00000040)) >> 6;
    *timestamptgt1intstat = (localVal & ((uint32_t)0x00000020)) >> 5;
    *finetgtintstat = (localVal & ((uint32_t)0x00000010)) >> 4;
    *swintstat = (localVal & ((uint32_t)0x00000008)) >> 3;
    *cryptintstat = (localVal & ((uint32_t)0x00000004)) >> 2;
    *slpintstat = (localVal & ((uint32_t)0x00000002)) >> 1;
    *clknintstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_intstat1_fifointstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t bt_intstat1_timestamptgt3intstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t bt_intstat1_timestamptgt2intstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t bt_intstat1_timestamptgt1intstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t bt_intstat1_finetgtintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t bt_intstat1_swintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t bt_intstat1_cryptintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t bt_intstat1_slpintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t bt_intstat1_clknintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTSTAT1_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief INTACK1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15           FIFOINTACK   0
 *     07   TIMESTAMPTGT3INTACK   0
 *     06   TIMESTAMPTGT2INTACK   0
 *     05   TIMESTAMPTGT1INTACK   0
 *     04        FINETGTINTACK   0
 *     03             SWINTACK   0
 *     02          CRYPTINTACK   0
 *     01            SLPINTACK   0
 *     00           CLKNINTACK   0
 * </pre>
 */
#define BT_INTACK1_ADDR   0x50090420
#define BT_INTACK1_OFFSET 0x00000020
#define BT_INTACK1_INDEX  0x00000008
#define BT_INTACK1_RESET  0x00000000

__INLINE uint32_t bt_intack1_get(void)
{
    return REG_BT_RD(BT_INTACK1_ADDR);
}

__INLINE void bt_intack1_clear(uint32_t value)
{
    REG_BT_WR(BT_INTACK1_ADDR, value);
}

// field definitions
#define BT_FIFOINTACK_BIT             ((uint32_t)0x00008000)
#define BT_FIFOINTACK_POS             15
#define BT_TIMESTAMPTGT3INTACK_BIT    ((uint32_t)0x00000080)
#define BT_TIMESTAMPTGT3INTACK_POS    7
#define BT_TIMESTAMPTGT2INTACK_BIT    ((uint32_t)0x00000040)
#define BT_TIMESTAMPTGT2INTACK_POS    6
#define BT_TIMESTAMPTGT1INTACK_BIT    ((uint32_t)0x00000020)
#define BT_TIMESTAMPTGT1INTACK_POS    5
#define BT_FINETGTINTACK_BIT          ((uint32_t)0x00000010)
#define BT_FINETGTINTACK_POS          4
#define BT_SWINTACK_BIT               ((uint32_t)0x00000008)
#define BT_SWINTACK_POS               3
#define BT_CRYPTINTACK_BIT            ((uint32_t)0x00000004)
#define BT_CRYPTINTACK_POS            2
#define BT_SLPINTACK_BIT              ((uint32_t)0x00000002)
#define BT_SLPINTACK_POS              1
#define BT_CLKNINTACK_BIT             ((uint32_t)0x00000001)
#define BT_CLKNINTACK_POS             0

#define BT_FIFOINTACK_RST             0x0
#define BT_TIMESTAMPTGT3INTACK_RST    0x0
#define BT_TIMESTAMPTGT2INTACK_RST    0x0
#define BT_TIMESTAMPTGT1INTACK_RST    0x0
#define BT_FINETGTINTACK_RST          0x0
#define BT_SWINTACK_RST               0x0
#define BT_CRYPTINTACK_RST            0x0
#define BT_SLPINTACK_RST              0x0
#define BT_CLKNINTACK_RST             0x0

__INLINE void bt_intack1_pack(uint8_t fifointack, uint8_t timestamptgt3intack, uint8_t timestamptgt2intack, uint8_t timestamptgt1intack, uint8_t finetgtintack, uint8_t swintack, uint8_t cryptintack, uint8_t slpintack, uint8_t clknintack)
{
    ASSERT_ERR((((uint32_t)fifointack << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgt3intack << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgt2intack << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgt1intack << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)finetgtintack << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)swintack << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)cryptintack << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)slpintack << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)clknintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, ((uint32_t)fifointack << 15) | ((uint32_t)timestamptgt3intack << 7) | ((uint32_t)timestamptgt2intack << 6) | ((uint32_t)timestamptgt1intack << 5) | ((uint32_t)finetgtintack << 4) | ((uint32_t)swintack << 3) | ((uint32_t)cryptintack << 2) | ((uint32_t)slpintack << 1) | ((uint32_t)clknintack << 0));
}

__INLINE void bt_intack1_unpack(uint8_t *fifointack, uint8_t *timestamptgt3intack, uint8_t *timestamptgt2intack, uint8_t *timestamptgt1intack, uint8_t *finetgtintack, uint8_t *swintack, uint8_t *cryptintack, uint8_t *slpintack, uint8_t *clknintack)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);

    *fifointack = (localVal & ((uint32_t)0x00008000)) >> 15;
    *timestamptgt3intack = (localVal & ((uint32_t)0x00000080)) >> 7;
    *timestamptgt2intack = (localVal & ((uint32_t)0x00000040)) >> 6;
    *timestamptgt1intack = (localVal & ((uint32_t)0x00000020)) >> 5;
    *finetgtintack = (localVal & ((uint32_t)0x00000010)) >> 4;
    *swintack = (localVal & ((uint32_t)0x00000008)) >> 3;
    *cryptintack = (localVal & ((uint32_t)0x00000004)) >> 2;
    *slpintack = (localVal & ((uint32_t)0x00000002)) >> 1;
    *clknintack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_intack1_fifointack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_intack1_fifointack_clearf(uint8_t fifointack)
{
    ASSERT_ERR((((uint32_t)fifointack << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)fifointack << 15);
}

__INLINE uint8_t bt_intack1_timestamptgt3intack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_intack1_timestamptgt3intack_clearf(uint8_t timestamptgt3intack)
{
    ASSERT_ERR((((uint32_t)timestamptgt3intack << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)timestamptgt3intack << 7);
}

__INLINE uint8_t bt_intack1_timestamptgt2intack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bt_intack1_timestamptgt2intack_clearf(uint8_t timestamptgt2intack)
{
    ASSERT_ERR((((uint32_t)timestamptgt2intack << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)timestamptgt2intack << 6);
}

__INLINE uint8_t bt_intack1_timestamptgt1intack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_intack1_timestamptgt1intack_clearf(uint8_t timestamptgt1intack)
{
    ASSERT_ERR((((uint32_t)timestamptgt1intack << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)timestamptgt1intack << 5);
}

__INLINE uint8_t bt_intack1_finetgtintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_intack1_finetgtintack_clearf(uint8_t finetgtintack)
{
    ASSERT_ERR((((uint32_t)finetgtintack << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)finetgtintack << 4);
}

__INLINE uint8_t bt_intack1_swintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_intack1_swintack_clearf(uint8_t swintack)
{
    ASSERT_ERR((((uint32_t)swintack << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)swintack << 3);
}

__INLINE uint8_t bt_intack1_cryptintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_intack1_cryptintack_clearf(uint8_t cryptintack)
{
    ASSERT_ERR((((uint32_t)cryptintack << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)cryptintack << 2);
}

__INLINE uint8_t bt_intack1_slpintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_intack1_slpintack_clearf(uint8_t slpintack)
{
    ASSERT_ERR((((uint32_t)slpintack << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)slpintack << 1);
}

__INLINE uint8_t bt_intack1_clknintack_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_INTACK1_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_intack1_clknintack_clearf(uint8_t clknintack)
{
    ASSERT_ERR((((uint32_t)clknintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_INTACK1_ADDR, (uint32_t)clknintack << 0);
}

/**
 * @brief ACTFIFOSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28          SKIP_ET_IDX   0x0
 *  27:24       CURRENT_ET_IDX   0x0
 *     15              ACTFLAG   0
 *     04            RXINTSTAT   0
 *     02       SKIPACTINTSTAT   0
 *     01        ENDACTINTSTAT   0
 *     00      STARTACTINTSTAT   0
 * </pre>
 */
#define BT_ACTFIFOSTAT_ADDR   0x50090424
#define BT_ACTFIFOSTAT_OFFSET 0x00000024
#define BT_ACTFIFOSTAT_INDEX  0x00000009
#define BT_ACTFIFOSTAT_RESET  0x00000000

__INLINE uint32_t bt_actfifostat_get(void)
{
    return REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
}

// field definitions
#define BT_SKIP_ET_IDX_MASK       ((uint32_t)0xF0000000)
#define BT_SKIP_ET_IDX_LSB        28
#define BT_SKIP_ET_IDX_WIDTH      ((uint32_t)0x00000004)
#define BT_CURRENT_ET_IDX_MASK    ((uint32_t)0x0F000000)
#define BT_CURRENT_ET_IDX_LSB     24
#define BT_CURRENT_ET_IDX_WIDTH   ((uint32_t)0x00000004)
#define BT_ACTFLAG_BIT            ((uint32_t)0x00008000)
#define BT_ACTFLAG_POS            15
#define BT_RXINTSTAT_BIT          ((uint32_t)0x00000010)
#define BT_RXINTSTAT_POS          4
#define BT_SKIPACTINTSTAT_BIT     ((uint32_t)0x00000004)
#define BT_SKIPACTINTSTAT_POS     2
#define BT_ENDACTINTSTAT_BIT      ((uint32_t)0x00000002)
#define BT_ENDACTINTSTAT_POS      1
#define BT_STARTACTINTSTAT_BIT    ((uint32_t)0x00000001)
#define BT_STARTACTINTSTAT_POS    0

#define BT_SKIP_ET_IDX_RST        0x0
#define BT_CURRENT_ET_IDX_RST     0x0
#define BT_ACTFLAG_RST            0x0
#define BT_RXINTSTAT_RST          0x0
#define BT_SKIPACTINTSTAT_RST     0x0
#define BT_ENDACTINTSTAT_RST      0x0
#define BT_STARTACTINTSTAT_RST    0x0

__INLINE void bt_actfifostat_unpack(uint8_t *skipetidx, uint8_t *currentetidx, uint8_t *actflag, uint8_t *rxintstat, uint8_t *skipactintstat, uint8_t *endactintstat, uint8_t *startactintstat)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);

    *skipetidx = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *currentetidx = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *actflag = (localVal & ((uint32_t)0x00008000)) >> 15;
    *rxintstat = (localVal & ((uint32_t)0x00000010)) >> 4;
    *skipactintstat = (localVal & ((uint32_t)0x00000004)) >> 2;
    *endactintstat = (localVal & ((uint32_t)0x00000002)) >> 1;
    *startactintstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_actfifostat_skip_et_idx_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE uint8_t bt_actfifostat_current_et_idx_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE uint8_t bt_actfifostat_actflag_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t bt_actfifostat_rxintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t bt_actfifostat_skipactintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t bt_actfifostat_endactintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t bt_actfifostat_startactintstat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief CURRENTRXDESCPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00     CURRENTRXDESCPTR   0x0
 * </pre>
 */
#define BT_CURRENTRXDESCPTR_ADDR   0x50090428
#define BT_CURRENTRXDESCPTR_OFFSET 0x00000028
#define BT_CURRENTRXDESCPTR_INDEX  0x0000000A
#define BT_CURRENTRXDESCPTR_RESET  0x00000000

__INLINE uint32_t bt_currentrxdescptr_get(void)
{
    return REG_BT_RD(BT_CURRENTRXDESCPTR_ADDR);
}

__INLINE void bt_currentrxdescptr_set(uint32_t value)
{
    REG_BT_WR(BT_CURRENTRXDESCPTR_ADDR, value);
}

// field definitions
#define BT_CURRENTRXDESCPTR_MASK   ((uint32_t)0x00003FFF)
#define BT_CURRENTRXDESCPTR_LSB    0
#define BT_CURRENTRXDESCPTR_WIDTH  ((uint32_t)0x0000000E)

#define BT_CURRENTRXDESCPTR_RST    0x0

__INLINE uint16_t bt_currentrxdescptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_CURRENTRXDESCPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_currentrxdescptr_setf(uint16_t currentrxdescptr)
{
    ASSERT_ERR((((uint32_t)currentrxdescptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_CURRENTRXDESCPTR_ADDR, (uint32_t)currentrxdescptr << 0);
}

/**
 * @brief ETPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00                ETPTR   0x0
 * </pre>
 */
#define BT_ETPTR_ADDR   0x5009042C
#define BT_ETPTR_OFFSET 0x0000002C
#define BT_ETPTR_INDEX  0x0000000B
#define BT_ETPTR_RESET  0x00000000

__INLINE uint32_t bt_etptr_get(void)
{
    return REG_BT_RD(BT_ETPTR_ADDR);
}

__INLINE void bt_etptr_set(uint32_t value)
{
    REG_BT_WR(BT_ETPTR_ADDR, value);
}

// field definitions
#define BT_ETPTR_MASK   ((uint32_t)0x00003FFF)
#define BT_ETPTR_LSB    0
#define BT_ETPTR_WIDTH  ((uint32_t)0x0000000E)

#define BT_ETPTR_RST    0x0

__INLINE uint16_t bt_etptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ETPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_etptr_setf(uint16_t etptr)
{
    ASSERT_ERR((((uint32_t)etptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_ETPTR_ADDR, (uint32_t)etptr << 0);
}

/**
 * @brief DEEPSLCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           EXTWKUPDSB   0
 *     15      DEEP_SLEEP_STAT   0
 *     03   DEEP_SLEEP_CORR_EN   0
 *     02        DEEP_SLEEP_ON   0
 *     01       RADIO_SLEEP_EN   0
 *     00         OSC_SLEEP_EN   0
 * </pre>
 */
#define BT_DEEPSLCNTL_ADDR   0x50090430
#define BT_DEEPSLCNTL_OFFSET 0x00000030
#define BT_DEEPSLCNTL_INDEX  0x0000000C
#define BT_DEEPSLCNTL_RESET  0x00000000

__INLINE uint32_t bt_deepslcntl_get(void)
{
    return REG_BT_RD(BT_DEEPSLCNTL_ADDR);
}

__INLINE void bt_deepslcntl_set(uint32_t value)
{
    REG_BT_WR(BT_DEEPSLCNTL_ADDR, value);
}

// field definitions
#define BT_EXTWKUPDSB_BIT            ((uint32_t)0x80000000)
#define BT_EXTWKUPDSB_POS            31
#define BT_DEEP_SLEEP_STAT_BIT       ((uint32_t)0x00008000)
#define BT_DEEP_SLEEP_STAT_POS       15
#define BT_DEEP_SLEEP_CORR_EN_BIT    ((uint32_t)0x00000008)
#define BT_DEEP_SLEEP_CORR_EN_POS    3
#define BT_DEEP_SLEEP_ON_BIT         ((uint32_t)0x00000004)
#define BT_DEEP_SLEEP_ON_POS         2
#define BT_RADIO_SLEEP_EN_BIT        ((uint32_t)0x00000002)
#define BT_RADIO_SLEEP_EN_POS        1
#define BT_OSC_SLEEP_EN_BIT          ((uint32_t)0x00000001)
#define BT_OSC_SLEEP_EN_POS          0

#define BT_EXTWKUPDSB_RST            0x0
#define BT_DEEP_SLEEP_STAT_RST       0x0
#define BT_DEEP_SLEEP_CORR_EN_RST    0x0
#define BT_DEEP_SLEEP_ON_RST         0x0
#define BT_RADIO_SLEEP_EN_RST        0x0
#define BT_OSC_SLEEP_EN_RST          0x0

__INLINE void bt_deepslcntl_pack(uint8_t extwkupdsb, uint8_t deepsleepcorren, uint8_t deepsleepon, uint8_t radiosleepen, uint8_t oscsleepen)
{
    ASSERT_ERR((((uint32_t)extwkupdsb << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)deepsleepcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)deepsleepon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)radiosleepen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)oscsleepen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_DEEPSLCNTL_ADDR, ((uint32_t)extwkupdsb << 31) | ((uint32_t)deepsleepcorren << 3) | ((uint32_t)deepsleepon << 2) | ((uint32_t)radiosleepen << 1) | ((uint32_t)oscsleepen << 0));
}

__INLINE void bt_deepslcntl_unpack(uint8_t *extwkupdsb, uint8_t *deepsleepstat, uint8_t *deepsleepcorren, uint8_t *deepsleepon, uint8_t *radiosleepen, uint8_t *oscsleepen)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLCNTL_ADDR);

    *extwkupdsb = (localVal & ((uint32_t)0x80000000)) >> 31;
    *deepsleepstat = (localVal & ((uint32_t)0x00008000)) >> 15;
    *deepsleepcorren = (localVal & ((uint32_t)0x00000008)) >> 3;
    *deepsleepon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *radiosleepen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *oscsleepen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_deepslcntl_extwkupdsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_deepslcntl_extwkupdsb_setf(uint8_t extwkupdsb)
{
    ASSERT_ERR((((uint32_t)extwkupdsb << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_DEEPSLCNTL_ADDR, (REG_BT_RD(BT_DEEPSLCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)extwkupdsb << 31));
}

__INLINE uint8_t bt_deepslcntl_deep_sleep_stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t bt_deepslcntl_deep_sleep_corr_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_deepslcntl_deep_sleep_corr_en_setf(uint8_t deepsleepcorren)
{
    ASSERT_ERR((((uint32_t)deepsleepcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_DEEPSLCNTL_ADDR, (REG_BT_RD(BT_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)deepsleepcorren << 3));
}

__INLINE uint8_t bt_deepslcntl_deep_sleep_on_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_deepslcntl_deep_sleep_on_setf(uint8_t deepsleepon)
{
    ASSERT_ERR((((uint32_t)deepsleepon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_DEEPSLCNTL_ADDR, (REG_BT_RD(BT_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)deepsleepon << 2));
}

__INLINE uint8_t bt_deepslcntl_radio_sleep_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_deepslcntl_radio_sleep_en_setf(uint8_t radiosleepen)
{
    ASSERT_ERR((((uint32_t)radiosleepen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_DEEPSLCNTL_ADDR, (REG_BT_RD(BT_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)radiosleepen << 1));
}

__INLINE uint8_t bt_deepslcntl_osc_sleep_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_deepslcntl_osc_sleep_en_setf(uint8_t oscsleepen)
{
    ASSERT_ERR((((uint32_t)oscsleepen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_DEEPSLCNTL_ADDR, (REG_BT_RD(BT_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)oscsleepen << 0));
}

/**
 * @brief DEEPSLWKUP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           DEEPSLTIME   0x0
 * </pre>
 */
#define BT_DEEPSLWKUP_ADDR   0x50090434
#define BT_DEEPSLWKUP_OFFSET 0x00000034
#define BT_DEEPSLWKUP_INDEX  0x0000000D
#define BT_DEEPSLWKUP_RESET  0x00000000

__INLINE uint32_t bt_deepslwkup_get(void)
{
    return REG_BT_RD(BT_DEEPSLWKUP_ADDR);
}

__INLINE void bt_deepslwkup_set(uint32_t value)
{
    REG_BT_WR(BT_DEEPSLWKUP_ADDR, value);
}

// field definitions
#define BT_DEEPSLTIME_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_DEEPSLTIME_LSB    0
#define BT_DEEPSLTIME_WIDTH  ((uint32_t)0x00000020)

#define BT_DEEPSLTIME_RST    0x0

__INLINE uint32_t bt_deepslwkup_deepsltime_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLWKUP_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_deepslwkup_deepsltime_setf(uint32_t deepsltime)
{
    ASSERT_ERR((((uint32_t)deepsltime << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BT_WR(BT_DEEPSLWKUP_ADDR, (uint32_t)deepsltime << 0);
}

/**
 * @brief DEEPSLSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            DEEPSLDUR   0x0
 * </pre>
 */
#define BT_DEEPSLSTAT_ADDR   0x50090438
#define BT_DEEPSLSTAT_OFFSET 0x00000038
#define BT_DEEPSLSTAT_INDEX  0x0000000E
#define BT_DEEPSLSTAT_RESET  0x00000000

__INLINE uint32_t bt_deepslstat_get(void)
{
    return REG_BT_RD(BT_DEEPSLSTAT_ADDR);
}

// field definitions
#define BT_DEEPSLDUR_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_DEEPSLDUR_LSB    0
#define BT_DEEPSLDUR_WIDTH  ((uint32_t)0x00000020)

#define BT_DEEPSLDUR_RST    0x0

__INLINE uint32_t bt_deepslstat_deepsldur_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEEPSLSTAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ENBPRESET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:21                TWEXT   0xA0
 *  20:10                TWOSC   0xA0
 *  09:00                 TWRM   0x20
 * </pre>
 */
#define BT_ENBPRESET_ADDR   0x5009043C
#define BT_ENBPRESET_OFFSET 0x0000003C
#define BT_ENBPRESET_INDEX  0x0000000F
#define BT_ENBPRESET_RESET  0x14028020

__INLINE uint32_t bt_enbpreset_get(void)
{
    return REG_BT_RD(BT_ENBPRESET_ADDR);
}

__INLINE void bt_enbpreset_set(uint32_t value)
{
    REG_BT_WR(BT_ENBPRESET_ADDR, value);
}

// field definitions
#define BT_TWEXT_MASK   ((uint32_t)0xFFE00000)
#define BT_TWEXT_LSB    21
#define BT_TWEXT_WIDTH  ((uint32_t)0x0000000B)
#define BT_TWOSC_MASK   ((uint32_t)0x001FFC00)
#define BT_TWOSC_LSB    10
#define BT_TWOSC_WIDTH  ((uint32_t)0x0000000B)
#define BT_TWRM_MASK    ((uint32_t)0x000003FF)
#define BT_TWRM_LSB     0
#define BT_TWRM_WIDTH   ((uint32_t)0x0000000A)

#define BT_TWEXT_RST    0xA0
#define BT_TWOSC_RST    0xA0
#define BT_TWRM_RST     0x20

__INLINE void bt_enbpreset_pack(uint16_t twext, uint16_t twosc, uint16_t twrm)
{
    ASSERT_ERR((((uint32_t)twext << 21) & ~((uint32_t)0xFFE00000)) == 0);
    ASSERT_ERR((((uint32_t)twosc << 10) & ~((uint32_t)0x001FFC00)) == 0);
    ASSERT_ERR((((uint32_t)twrm << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BT_WR(BT_ENBPRESET_ADDR, ((uint32_t)twext << 21) | ((uint32_t)twosc << 10) | ((uint32_t)twrm << 0));
}

__INLINE void bt_enbpreset_unpack(uint16_t *twext, uint16_t *twosc, uint16_t *twrm)
{
    uint32_t localVal = REG_BT_RD(BT_ENBPRESET_ADDR);

    *twext = (localVal & ((uint32_t)0xFFE00000)) >> 21;
    *twosc = (localVal & ((uint32_t)0x001FFC00)) >> 10;
    *twrm = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t bt_enbpreset_twext_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0xFFE00000)) >> 21);
}

__INLINE void bt_enbpreset_twext_setf(uint16_t twext)
{
    ASSERT_ERR((((uint32_t)twext << 21) & ~((uint32_t)0xFFE00000)) == 0);
    REG_BT_WR(BT_ENBPRESET_ADDR, (REG_BT_RD(BT_ENBPRESET_ADDR) & ~((uint32_t)0xFFE00000)) | ((uint32_t)twext << 21));
}

__INLINE uint16_t bt_enbpreset_twosc_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0x001FFC00)) >> 10);
}

__INLINE void bt_enbpreset_twosc_setf(uint16_t twosc)
{
    ASSERT_ERR((((uint32_t)twosc << 10) & ~((uint32_t)0x001FFC00)) == 0);
    REG_BT_WR(BT_ENBPRESET_ADDR, (REG_BT_RD(BT_ENBPRESET_ADDR) & ~((uint32_t)0x001FFC00)) | ((uint32_t)twosc << 10));
}

__INLINE uint16_t bt_enbpreset_twrm_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void bt_enbpreset_twrm_setf(uint16_t twrm)
{
    ASSERT_ERR((((uint32_t)twrm << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BT_WR(BT_ENBPRESET_ADDR, (REG_BT_RD(BT_ENBPRESET_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)twrm << 0));
}

/**
 * @brief FINECNTCORR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00          FINECNTCORR   0x0
 * </pre>
 */
#define BT_FINECNTCORR_ADDR   0x50090440
#define BT_FINECNTCORR_OFFSET 0x00000040
#define BT_FINECNTCORR_INDEX  0x00000010
#define BT_FINECNTCORR_RESET  0x00000000

__INLINE uint32_t bt_finecntcorr_get(void)
{
    return REG_BT_RD(BT_FINECNTCORR_ADDR);
}

__INLINE void bt_finecntcorr_set(uint32_t value)
{
    REG_BT_WR(BT_FINECNTCORR_ADDR, value);
}

// field definitions
#define BT_FINECNTCORR_MASK   ((uint32_t)0x000003FF)
#define BT_FINECNTCORR_LSB    0
#define BT_FINECNTCORR_WIDTH  ((uint32_t)0x0000000A)

#define BT_FINECNTCORR_RST    0x0

__INLINE uint16_t bt_finecntcorr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_FINECNTCORR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_finecntcorr_setf(uint16_t finecntcorr)
{
    ASSERT_ERR((((uint32_t)finecntcorr << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BT_WR(BT_FINECNTCORR_ADDR, (uint32_t)finecntcorr << 0);
}

/**
 * @brief CLKNCNTCORR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            ABS_DELTA   0
 *  27:00          CLKNCNTCORR   0x0
 * </pre>
 */
#define BT_CLKNCNTCORR_ADDR   0x50090444
#define BT_CLKNCNTCORR_OFFSET 0x00000044
#define BT_CLKNCNTCORR_INDEX  0x00000011
#define BT_CLKNCNTCORR_RESET  0x00000000

__INLINE uint32_t bt_clkncntcorr_get(void)
{
    return REG_BT_RD(BT_CLKNCNTCORR_ADDR);
}

__INLINE void bt_clkncntcorr_set(uint32_t value)
{
    REG_BT_WR(BT_CLKNCNTCORR_ADDR, value);
}

// field definitions
#define BT_ABS_DELTA_BIT      ((uint32_t)0x80000000)
#define BT_ABS_DELTA_POS      31
#define BT_CLKNCNTCORR_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_CLKNCNTCORR_LSB    0
#define BT_CLKNCNTCORR_WIDTH  ((uint32_t)0x0000001C)

#define BT_ABS_DELTA_RST      0x0
#define BT_CLKNCNTCORR_RST    0x0

__INLINE void bt_clkncntcorr_pack(uint8_t absdelta, uint32_t clkncntcorr)
{
    ASSERT_ERR((((uint32_t)absdelta << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)clkncntcorr << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_CLKNCNTCORR_ADDR, ((uint32_t)absdelta << 31) | ((uint32_t)clkncntcorr << 0));
}

__INLINE void bt_clkncntcorr_unpack(uint8_t *absdelta, uint32_t *clkncntcorr)
{
    uint32_t localVal = REG_BT_RD(BT_CLKNCNTCORR_ADDR);

    *absdelta = (localVal & ((uint32_t)0x80000000)) >> 31;
    *clkncntcorr = (localVal & ((uint32_t)0x0FFFFFFF)) >> 0;
}

__INLINE uint8_t bt_clkncntcorr_abs_delta_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_CLKNCNTCORR_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_clkncntcorr_abs_delta_setf(uint8_t absdelta)
{
    ASSERT_ERR((((uint32_t)absdelta << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_CLKNCNTCORR_ADDR, (REG_BT_RD(BT_CLKNCNTCORR_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)absdelta << 31));
}

__INLINE uint32_t bt_clkncntcorr_clkncntcorr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_CLKNCNTCORR_ADDR);
    return ((localVal & ((uint32_t)0x0FFFFFFF)) >> 0);
}

__INLINE void bt_clkncntcorr_clkncntcorr_setf(uint32_t clkncntcorr)
{
    ASSERT_ERR((((uint32_t)clkncntcorr << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_CLKNCNTCORR_ADDR, (REG_BT_RD(BT_CLKNCNTCORR_ADDR) & ~((uint32_t)0x0FFFFFFF)) | ((uint32_t)clkncntcorr << 0));
}

/**
 * @brief DIAGCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31             DIAG3_EN   0
 *  30:24                DIAG3   0x0
 *     23             DIAG2_EN   0
 *  22:16                DIAG2   0x0
 *     15             DIAG1_EN   0
 *  14:08                DIAG1   0x0
 *     07             DIAG0_EN   0
 *  06:00                DIAG0   0x0
 * </pre>
 */
#define BT_DIAGCNTL_ADDR   0x50090450
#define BT_DIAGCNTL_OFFSET 0x00000050
#define BT_DIAGCNTL_INDEX  0x00000014
#define BT_DIAGCNTL_RESET  0x00000000

__INLINE uint32_t bt_diagcntl_get(void)
{
    return REG_BT_RD(BT_DIAGCNTL_ADDR);
}

__INLINE void bt_diagcntl_set(uint32_t value)
{
    REG_BT_WR(BT_DIAGCNTL_ADDR, value);
}

// field definitions
#define BT_DIAG3_EN_BIT    ((uint32_t)0x80000000)
#define BT_DIAG3_EN_POS    31
#define BT_DIAG3_MASK      ((uint32_t)0x7F000000)
#define BT_DIAG3_LSB       24
#define BT_DIAG3_WIDTH     ((uint32_t)0x00000007)
#define BT_DIAG2_EN_BIT    ((uint32_t)0x00800000)
#define BT_DIAG2_EN_POS    23
#define BT_DIAG2_MASK      ((uint32_t)0x007F0000)
#define BT_DIAG2_LSB       16
#define BT_DIAG2_WIDTH     ((uint32_t)0x00000007)
#define BT_DIAG1_EN_BIT    ((uint32_t)0x00008000)
#define BT_DIAG1_EN_POS    15
#define BT_DIAG1_MASK      ((uint32_t)0x00007F00)
#define BT_DIAG1_LSB       8
#define BT_DIAG1_WIDTH     ((uint32_t)0x00000007)
#define BT_DIAG0_EN_BIT    ((uint32_t)0x00000080)
#define BT_DIAG0_EN_POS    7
#define BT_DIAG0_MASK      ((uint32_t)0x0000007F)
#define BT_DIAG0_LSB       0
#define BT_DIAG0_WIDTH     ((uint32_t)0x00000007)

#define BT_DIAG3_EN_RST    0x0
#define BT_DIAG3_RST       0x0
#define BT_DIAG2_EN_RST    0x0
#define BT_DIAG2_RST       0x0
#define BT_DIAG1_EN_RST    0x0
#define BT_DIAG1_RST       0x0
#define BT_DIAG0_EN_RST    0x0
#define BT_DIAG0_RST       0x0

__INLINE void bt_diagcntl_pack(uint8_t diag3en, uint8_t diag3, uint8_t diag2en, uint8_t diag2, uint8_t diag1en, uint8_t diag1, uint8_t diag0en, uint8_t diag0)
{
    ASSERT_ERR((((uint32_t)diag3en << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)diag3 << 24) & ~((uint32_t)0x7F000000)) == 0);
    ASSERT_ERR((((uint32_t)diag2en << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)diag2 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)diag1en << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)diag1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)diag0en << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)diag0 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, ((uint32_t)diag3en << 31) | ((uint32_t)diag3 << 24) | ((uint32_t)diag2en << 23) | ((uint32_t)diag2 << 16) | ((uint32_t)diag1en << 15) | ((uint32_t)diag1 << 8) | ((uint32_t)diag0en << 7) | ((uint32_t)diag0 << 0));
}

__INLINE void bt_diagcntl_unpack(uint8_t *diag3en, uint8_t *diag3, uint8_t *diag2en, uint8_t *diag2, uint8_t *diag1en, uint8_t *diag1, uint8_t *diag0en, uint8_t *diag0)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);

    *diag3en = (localVal & ((uint32_t)0x80000000)) >> 31;
    *diag3 = (localVal & ((uint32_t)0x7F000000)) >> 24;
    *diag2en = (localVal & ((uint32_t)0x00800000)) >> 23;
    *diag2 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *diag1en = (localVal & ((uint32_t)0x00008000)) >> 15;
    *diag1 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *diag0en = (localVal & ((uint32_t)0x00000080)) >> 7;
    *diag0 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t bt_diagcntl_diag3_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_diagcntl_diag3_en_setf(uint8_t diag3en)
{
    ASSERT_ERR((((uint32_t)diag3en << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)diag3en << 31));
}

__INLINE uint8_t bt_diagcntl_diag3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x7F000000)) >> 24);
}

__INLINE void bt_diagcntl_diag3_setf(uint8_t diag3)
{
    ASSERT_ERR((((uint32_t)diag3 << 24) & ~((uint32_t)0x7F000000)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x7F000000)) | ((uint32_t)diag3 << 24));
}

__INLINE uint8_t bt_diagcntl_diag2_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bt_diagcntl_diag2_en_setf(uint8_t diag2en)
{
    ASSERT_ERR((((uint32_t)diag2en << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)diag2en << 23));
}

__INLINE uint8_t bt_diagcntl_diag2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void bt_diagcntl_diag2_setf(uint8_t diag2)
{
    ASSERT_ERR((((uint32_t)diag2 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)diag2 << 16));
}

__INLINE uint8_t bt_diagcntl_diag1_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_diagcntl_diag1_en_setf(uint8_t diag1en)
{
    ASSERT_ERR((((uint32_t)diag1en << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)diag1en << 15));
}

__INLINE uint8_t bt_diagcntl_diag1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void bt_diagcntl_diag1_setf(uint8_t diag1)
{
    ASSERT_ERR((((uint32_t)diag1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)diag1 << 8));
}

__INLINE uint8_t bt_diagcntl_diag0_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_diagcntl_diag0_en_setf(uint8_t diag0en)
{
    ASSERT_ERR((((uint32_t)diag0en << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)diag0en << 7));
}

__INLINE uint8_t bt_diagcntl_diag0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void bt_diagcntl_diag0_setf(uint8_t diag0)
{
    ASSERT_ERR((((uint32_t)diag0 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_DIAGCNTL_ADDR, (REG_BT_RD(BT_DIAGCNTL_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)diag0 << 0));
}

/**
 * @brief DIAGSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            DIAG3STAT   0x0
 *  23:16            DIAG2STAT   0x0
 *  15:08            DIAG1STAT   0x0
 *  07:00            DIAG0STAT   0x0
 * </pre>
 */
#define BT_DIAGSTAT_ADDR   0x50090454
#define BT_DIAGSTAT_OFFSET 0x00000054
#define BT_DIAGSTAT_INDEX  0x00000015
#define BT_DIAGSTAT_RESET  0x00000000

__INLINE uint32_t bt_diagstat_get(void)
{
    return REG_BT_RD(BT_DIAGSTAT_ADDR);
}

// field definitions
#define BT_DIAG3STAT_MASK   ((uint32_t)0xFF000000)
#define BT_DIAG3STAT_LSB    24
#define BT_DIAG3STAT_WIDTH  ((uint32_t)0x00000008)
#define BT_DIAG2STAT_MASK   ((uint32_t)0x00FF0000)
#define BT_DIAG2STAT_LSB    16
#define BT_DIAG2STAT_WIDTH  ((uint32_t)0x00000008)
#define BT_DIAG1STAT_MASK   ((uint32_t)0x0000FF00)
#define BT_DIAG1STAT_LSB    8
#define BT_DIAG1STAT_WIDTH  ((uint32_t)0x00000008)
#define BT_DIAG0STAT_MASK   ((uint32_t)0x000000FF)
#define BT_DIAG0STAT_LSB    0
#define BT_DIAG0STAT_WIDTH  ((uint32_t)0x00000008)

#define BT_DIAG3STAT_RST    0x0
#define BT_DIAG2STAT_RST    0x0
#define BT_DIAG1STAT_RST    0x0
#define BT_DIAG0STAT_RST    0x0

__INLINE void bt_diagstat_unpack(uint8_t *diag3stat, uint8_t *diag2stat, uint8_t *diag1stat, uint8_t *diag0stat)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGSTAT_ADDR);

    *diag3stat = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *diag2stat = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *diag1stat = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *diag0stat = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bt_diagstat_diag3stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t bt_diagstat_diag2stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t bt_diagstat_diag1stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t bt_diagstat_diag0stat_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief DEBUGADDMAX register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           REG_ADDMAX   0x0
 *  15:00            EM_ADDMAX   0x0
 * </pre>
 */
#define BT_DEBUGADDMAX_ADDR   0x50090458
#define BT_DEBUGADDMAX_OFFSET 0x00000058
#define BT_DEBUGADDMAX_INDEX  0x00000016
#define BT_DEBUGADDMAX_RESET  0x00000000

__INLINE uint32_t bt_debugaddmax_get(void)
{
    return REG_BT_RD(BT_DEBUGADDMAX_ADDR);
}

__INLINE void bt_debugaddmax_set(uint32_t value)
{
    REG_BT_WR(BT_DEBUGADDMAX_ADDR, value);
}

// field definitions
#define BT_REG_ADDMAX_MASK   ((uint32_t)0xFFFF0000)
#define BT_REG_ADDMAX_LSB    16
#define BT_REG_ADDMAX_WIDTH  ((uint32_t)0x00000010)
#define BT_EM_ADDMAX_MASK    ((uint32_t)0x0000FFFF)
#define BT_EM_ADDMAX_LSB     0
#define BT_EM_ADDMAX_WIDTH   ((uint32_t)0x00000010)

#define BT_REG_ADDMAX_RST    0x0
#define BT_EM_ADDMAX_RST     0x0

__INLINE void bt_debugaddmax_pack(uint16_t regaddmax, uint16_t emaddmax)
{
    ASSERT_ERR((((uint32_t)regaddmax << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)emaddmax << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_DEBUGADDMAX_ADDR, ((uint32_t)regaddmax << 16) | ((uint32_t)emaddmax << 0));
}

__INLINE void bt_debugaddmax_unpack(uint16_t *regaddmax, uint16_t *emaddmax)
{
    uint32_t localVal = REG_BT_RD(BT_DEBUGADDMAX_ADDR);

    *regaddmax = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *emaddmax = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_debugaddmax_reg_addmax_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEBUGADDMAX_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_debugaddmax_reg_addmax_setf(uint16_t regaddmax)
{
    ASSERT_ERR((((uint32_t)regaddmax << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_DEBUGADDMAX_ADDR, (REG_BT_RD(BT_DEBUGADDMAX_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)regaddmax << 16));
}

__INLINE uint16_t bt_debugaddmax_em_addmax_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEBUGADDMAX_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_debugaddmax_em_addmax_setf(uint16_t emaddmax)
{
    ASSERT_ERR((((uint32_t)emaddmax << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_DEBUGADDMAX_ADDR, (REG_BT_RD(BT_DEBUGADDMAX_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)emaddmax << 0));
}

/**
 * @brief DEBUGADDMIN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           REG_ADDMIN   0x0
 *  15:00            EM_ADDMIN   0x0
 * </pre>
 */
#define BT_DEBUGADDMIN_ADDR   0x5009045C
#define BT_DEBUGADDMIN_OFFSET 0x0000005C
#define BT_DEBUGADDMIN_INDEX  0x00000017
#define BT_DEBUGADDMIN_RESET  0x00000000

__INLINE uint32_t bt_debugaddmin_get(void)
{
    return REG_BT_RD(BT_DEBUGADDMIN_ADDR);
}

__INLINE void bt_debugaddmin_set(uint32_t value)
{
    REG_BT_WR(BT_DEBUGADDMIN_ADDR, value);
}

// field definitions
#define BT_REG_ADDMIN_MASK   ((uint32_t)0xFFFF0000)
#define BT_REG_ADDMIN_LSB    16
#define BT_REG_ADDMIN_WIDTH  ((uint32_t)0x00000010)
#define BT_EM_ADDMIN_MASK    ((uint32_t)0x0000FFFF)
#define BT_EM_ADDMIN_LSB     0
#define BT_EM_ADDMIN_WIDTH   ((uint32_t)0x00000010)

#define BT_REG_ADDMIN_RST    0x0
#define BT_EM_ADDMIN_RST     0x0

__INLINE void bt_debugaddmin_pack(uint16_t regaddmin, uint16_t emaddmin)
{
    ASSERT_ERR((((uint32_t)regaddmin << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)emaddmin << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_DEBUGADDMIN_ADDR, ((uint32_t)regaddmin << 16) | ((uint32_t)emaddmin << 0));
}

__INLINE void bt_debugaddmin_unpack(uint16_t *regaddmin, uint16_t *emaddmin)
{
    uint32_t localVal = REG_BT_RD(BT_DEBUGADDMIN_ADDR);

    *regaddmin = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *emaddmin = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_debugaddmin_reg_addmin_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEBUGADDMIN_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_debugaddmin_reg_addmin_setf(uint16_t regaddmin)
{
    ASSERT_ERR((((uint32_t)regaddmin << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_DEBUGADDMIN_ADDR, (REG_BT_RD(BT_DEBUGADDMIN_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)regaddmin << 16));
}

__INLINE uint16_t bt_debugaddmin_em_addmin_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_DEBUGADDMIN_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_debugaddmin_em_addmin_setf(uint16_t emaddmin)
{
    ASSERT_ERR((((uint32_t)emaddmin << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_DEBUGADDMIN_ADDR, (REG_BT_RD(BT_DEBUGADDMIN_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)emaddmin << 0));
}

/**
 * @brief ERRORTYPESTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     23           FIFOINTOVF   0
 *     22      LOCAL_SAM_ERROR   0
 *     21       PEER_SAM_ERROR   0
 *     20      RXBUF_PTR_ERROR   0
 *     19      TXBUF_PTR_ERROR   0
 *     18   RXDESC_EMPTY_ERROR   0
 *     17   TXDESC_EMPTY_ERROR   0
 *     16        CSATTNB_ERROR   0
 *     15       CSFORMAT_ERROR   0
 *     14          CHMAP_ERROR   0
 *     13    HOPUNDERRUN_ERROR   0
 *     12   FRM_CNTL_TIMER_ERROR   0
 *     11   FRM_CNTL_EMACC_ERROR   0
 *     10   FRM_CNTL_APFM_ERROR   0
 *     09   ACT_SCHDL_APFM_ERROR   0
 *     08   ACT_SCHDL_ENTRY_ERROR   0
 *     07   MWSCOEX_EMACC_ERROR   0
 *     06      PCM_EMACC_ERROR   0
 *     05    AUDIO_EMACC_ERROR   0
 *     04   RADIOCNTL_EMACC_ERROR   0
 *     03   PKTCNTL_EMACC_ERROR   0
 *     02      CRYPTMODE_ERROR   0
 *     01        RXCRYPT_ERROR   0
 *     00        TXCRYPT_ERROR   0
 * </pre>
 */
#define BT_ERRORTYPESTAT_ADDR   0x50090460
#define BT_ERRORTYPESTAT_OFFSET 0x00000060
#define BT_ERRORTYPESTAT_INDEX  0x00000018
#define BT_ERRORTYPESTAT_RESET  0x00000000

__INLINE uint32_t bt_errortypestat_get(void)
{
    return REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
}

// field definitions
#define BT_FIFOINTOVF_BIT               ((uint32_t)0x00800000)
#define BT_FIFOINTOVF_POS               23
#define BT_LOCAL_SAM_ERROR_BIT          ((uint32_t)0x00400000)
#define BT_LOCAL_SAM_ERROR_POS          22
#define BT_PEER_SAM_ERROR_BIT           ((uint32_t)0x00200000)
#define BT_PEER_SAM_ERROR_POS           21
#define BT_RXBUF_PTR_ERROR_BIT          ((uint32_t)0x00100000)
#define BT_RXBUF_PTR_ERROR_POS          20
#define BT_TXBUF_PTR_ERROR_BIT          ((uint32_t)0x00080000)
#define BT_TXBUF_PTR_ERROR_POS          19
#define BT_RXDESC_EMPTY_ERROR_BIT       ((uint32_t)0x00040000)
#define BT_RXDESC_EMPTY_ERROR_POS       18
#define BT_TXDESC_EMPTY_ERROR_BIT       ((uint32_t)0x00020000)
#define BT_TXDESC_EMPTY_ERROR_POS       17
#define BT_CSATTNB_ERROR_BIT            ((uint32_t)0x00010000)
#define BT_CSATTNB_ERROR_POS            16
#define BT_CSFORMAT_ERROR_BIT           ((uint32_t)0x00008000)
#define BT_CSFORMAT_ERROR_POS           15
#define BT_CHMAP_ERROR_BIT              ((uint32_t)0x00004000)
#define BT_CHMAP_ERROR_POS              14
#define BT_HOPUNDERRUN_ERROR_BIT        ((uint32_t)0x00002000)
#define BT_HOPUNDERRUN_ERROR_POS        13
#define BT_FRM_CNTL_TIMER_ERROR_BIT     ((uint32_t)0x00001000)
#define BT_FRM_CNTL_TIMER_ERROR_POS     12
#define BT_FRM_CNTL_EMACC_ERROR_BIT     ((uint32_t)0x00000800)
#define BT_FRM_CNTL_EMACC_ERROR_POS     11
#define BT_FRM_CNTL_APFM_ERROR_BIT      ((uint32_t)0x00000400)
#define BT_FRM_CNTL_APFM_ERROR_POS      10
#define BT_ACT_SCHDL_APFM_ERROR_BIT     ((uint32_t)0x00000200)
#define BT_ACT_SCHDL_APFM_ERROR_POS     9
#define BT_ACT_SCHDL_ENTRY_ERROR_BIT    ((uint32_t)0x00000100)
#define BT_ACT_SCHDL_ENTRY_ERROR_POS    8
#define BT_MWSCOEX_EMACC_ERROR_BIT      ((uint32_t)0x00000080)
#define BT_MWSCOEX_EMACC_ERROR_POS      7
#define BT_PCM_EMACC_ERROR_BIT          ((uint32_t)0x00000040)
#define BT_PCM_EMACC_ERROR_POS          6
#define BT_AUDIO_EMACC_ERROR_BIT        ((uint32_t)0x00000020)
#define BT_AUDIO_EMACC_ERROR_POS        5
#define BT_RADIOCNTL_EMACC_ERROR_BIT    ((uint32_t)0x00000010)
#define BT_RADIOCNTL_EMACC_ERROR_POS    4
#define BT_PKTCNTL_EMACC_ERROR_BIT      ((uint32_t)0x00000008)
#define BT_PKTCNTL_EMACC_ERROR_POS      3
#define BT_CRYPTMODE_ERROR_BIT          ((uint32_t)0x00000004)
#define BT_CRYPTMODE_ERROR_POS          2
#define BT_RXCRYPT_ERROR_BIT            ((uint32_t)0x00000002)
#define BT_RXCRYPT_ERROR_POS            1
#define BT_TXCRYPT_ERROR_BIT            ((uint32_t)0x00000001)
#define BT_TXCRYPT_ERROR_POS            0

#define BT_FIFOINTOVF_RST               0x0
#define BT_LOCAL_SAM_ERROR_RST          0x0
#define BT_PEER_SAM_ERROR_RST           0x0
#define BT_RXBUF_PTR_ERROR_RST          0x0
#define BT_TXBUF_PTR_ERROR_RST          0x0
#define BT_RXDESC_EMPTY_ERROR_RST       0x0
#define BT_TXDESC_EMPTY_ERROR_RST       0x0
#define BT_CSATTNB_ERROR_RST            0x0
#define BT_CSFORMAT_ERROR_RST           0x0
#define BT_CHMAP_ERROR_RST              0x0
#define BT_HOPUNDERRUN_ERROR_RST        0x0
#define BT_FRM_CNTL_TIMER_ERROR_RST     0x0
#define BT_FRM_CNTL_EMACC_ERROR_RST     0x0
#define BT_FRM_CNTL_APFM_ERROR_RST      0x0
#define BT_ACT_SCHDL_APFM_ERROR_RST     0x0
#define BT_ACT_SCHDL_ENTRY_ERROR_RST    0x0
#define BT_MWSCOEX_EMACC_ERROR_RST      0x0
#define BT_PCM_EMACC_ERROR_RST          0x0
#define BT_AUDIO_EMACC_ERROR_RST        0x0
#define BT_RADIOCNTL_EMACC_ERROR_RST    0x0
#define BT_PKTCNTL_EMACC_ERROR_RST      0x0
#define BT_CRYPTMODE_ERROR_RST          0x0
#define BT_RXCRYPT_ERROR_RST            0x0
#define BT_TXCRYPT_ERROR_RST            0x0

__INLINE void bt_errortypestat_unpack(uint8_t *fifointovf, uint8_t *localsamerror, uint8_t *peersamerror, uint8_t *rxbufptrerror, uint8_t *txbufptrerror, uint8_t *rxdescemptyerror, uint8_t *txdescemptyerror, uint8_t *csattnberror, uint8_t *csformaterror, uint8_t *chmaperror, uint8_t *hopunderrunerror, uint8_t *frmcntltimererror, uint8_t *frmcntlemaccerror, uint8_t *frmcntlapfmerror, uint8_t *actschdlapfmerror, uint8_t *actschdlentryerror, uint8_t *mwscoexemaccerror, uint8_t *pcmemaccerror, uint8_t *audioemaccerror, uint8_t *radiocntlemaccerror, uint8_t *pktcntlemaccerror, uint8_t *cryptmodeerror, uint8_t *rxcrypterror, uint8_t *txcrypterror)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);

    *fifointovf = (localVal & ((uint32_t)0x00800000)) >> 23;
    *localsamerror = (localVal & ((uint32_t)0x00400000)) >> 22;
    *peersamerror = (localVal & ((uint32_t)0x00200000)) >> 21;
    *rxbufptrerror = (localVal & ((uint32_t)0x00100000)) >> 20;
    *txbufptrerror = (localVal & ((uint32_t)0x00080000)) >> 19;
    *rxdescemptyerror = (localVal & ((uint32_t)0x00040000)) >> 18;
    *txdescemptyerror = (localVal & ((uint32_t)0x00020000)) >> 17;
    *csattnberror = (localVal & ((uint32_t)0x00010000)) >> 16;
    *csformaterror = (localVal & ((uint32_t)0x00008000)) >> 15;
    *chmaperror = (localVal & ((uint32_t)0x00004000)) >> 14;
    *hopunderrunerror = (localVal & ((uint32_t)0x00002000)) >> 13;
    *frmcntltimererror = (localVal & ((uint32_t)0x00001000)) >> 12;
    *frmcntlemaccerror = (localVal & ((uint32_t)0x00000800)) >> 11;
    *frmcntlapfmerror = (localVal & ((uint32_t)0x00000400)) >> 10;
    *actschdlapfmerror = (localVal & ((uint32_t)0x00000200)) >> 9;
    *actschdlentryerror = (localVal & ((uint32_t)0x00000100)) >> 8;
    *mwscoexemaccerror = (localVal & ((uint32_t)0x00000080)) >> 7;
    *pcmemaccerror = (localVal & ((uint32_t)0x00000040)) >> 6;
    *audioemaccerror = (localVal & ((uint32_t)0x00000020)) >> 5;
    *radiocntlemaccerror = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pktcntlemaccerror = (localVal & ((uint32_t)0x00000008)) >> 3;
    *cryptmodeerror = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxcrypterror = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txcrypterror = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_errortypestat_fifointovf_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE uint8_t bt_errortypestat_local_sam_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE uint8_t bt_errortypestat_peer_sam_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE uint8_t bt_errortypestat_rxbuf_ptr_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE uint8_t bt_errortypestat_txbuf_ptr_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE uint8_t bt_errortypestat_rxdesc_empty_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE uint8_t bt_errortypestat_txdesc_empty_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE uint8_t bt_errortypestat_csattnb_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint8_t bt_errortypestat_csformat_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t bt_errortypestat_chmap_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t bt_errortypestat_hopunderrun_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t bt_errortypestat_frm_cntl_timer_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t bt_errortypestat_frm_cntl_emacc_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t bt_errortypestat_frm_cntl_apfm_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t bt_errortypestat_act_schdl_apfm_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t bt_errortypestat_act_schdl_entry_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t bt_errortypestat_mwscoex_emacc_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t bt_errortypestat_pcm_emacc_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t bt_errortypestat_audio_emacc_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t bt_errortypestat_radiocntl_emacc_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t bt_errortypestat_pktcntl_emacc_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t bt_errortypestat_cryptmode_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t bt_errortypestat_rxcrypt_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t bt_errortypestat_txcrypt_error_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief SWPROFILING register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31             SWPROF31   0
 *     30             SWPROF30   0
 *     29             SWPROF29   0
 *     28             SWPROF28   0
 *     27             SWPROF27   0
 *     26             SWPROF26   0
 *     25             SWPROF25   0
 *     24             SWPROF24   0
 *     23             SWPROF23   0
 *     22             SWPROF22   0
 *     21             SWPROF21   0
 *     20             SWPROF20   0
 *     19             SWPROF19   0
 *     18             SWPROF18   0
 *     17             SWPROF17   0
 *     16             SWPROF16   0
 *     15             SWPROF15   0
 *     14             SWPROF14   0
 *     13             SWPROF13   0
 *     12             SWPROF12   0
 *     11             SWPROF11   0
 *     10             SWPROF10   0
 *     09              SWPROF9   0
 *     08              SWPROF8   0
 *     07              SWPROF7   0
 *     06              SWPROF6   0
 *     05              SWPROF5   0
 *     04              SWPROF4   0
 *     03              SWPROF3   0
 *     02              SWPROF2   0
 *     01              SWPROF1   0
 *     00              SWPROF0   0
 * </pre>
 */
#define BT_SWPROFILING_ADDR   0x50090464
#define BT_SWPROFILING_OFFSET 0x00000064
#define BT_SWPROFILING_INDEX  0x00000019
#define BT_SWPROFILING_RESET  0x00000000

__INLINE uint32_t bt_swprofiling_get(void)
{
    return REG_BT_RD(BT_SWPROFILING_ADDR);
}

__INLINE void bt_swprofiling_set(uint32_t value)
{
    REG_BT_WR(BT_SWPROFILING_ADDR, value);
}

// field definitions
#define BT_SWPROF31_BIT    ((uint32_t)0x80000000)
#define BT_SWPROF31_POS    31
#define BT_SWPROF30_BIT    ((uint32_t)0x40000000)
#define BT_SWPROF30_POS    30
#define BT_SWPROF29_BIT    ((uint32_t)0x20000000)
#define BT_SWPROF29_POS    29
#define BT_SWPROF28_BIT    ((uint32_t)0x10000000)
#define BT_SWPROF28_POS    28
#define BT_SWPROF27_BIT    ((uint32_t)0x08000000)
#define BT_SWPROF27_POS    27
#define BT_SWPROF26_BIT    ((uint32_t)0x04000000)
#define BT_SWPROF26_POS    26
#define BT_SWPROF25_BIT    ((uint32_t)0x02000000)
#define BT_SWPROF25_POS    25
#define BT_SWPROF24_BIT    ((uint32_t)0x01000000)
#define BT_SWPROF24_POS    24
#define BT_SWPROF23_BIT    ((uint32_t)0x00800000)
#define BT_SWPROF23_POS    23
#define BT_SWPROF22_BIT    ((uint32_t)0x00400000)
#define BT_SWPROF22_POS    22
#define BT_SWPROF21_BIT    ((uint32_t)0x00200000)
#define BT_SWPROF21_POS    21
#define BT_SWPROF20_BIT    ((uint32_t)0x00100000)
#define BT_SWPROF20_POS    20
#define BT_SWPROF19_BIT    ((uint32_t)0x00080000)
#define BT_SWPROF19_POS    19
#define BT_SWPROF18_BIT    ((uint32_t)0x00040000)
#define BT_SWPROF18_POS    18
#define BT_SWPROF17_BIT    ((uint32_t)0x00020000)
#define BT_SWPROF17_POS    17
#define BT_SWPROF16_BIT    ((uint32_t)0x00010000)
#define BT_SWPROF16_POS    16
#define BT_SWPROF15_BIT    ((uint32_t)0x00008000)
#define BT_SWPROF15_POS    15
#define BT_SWPROF14_BIT    ((uint32_t)0x00004000)
#define BT_SWPROF14_POS    14
#define BT_SWPROF13_BIT    ((uint32_t)0x00002000)
#define BT_SWPROF13_POS    13
#define BT_SWPROF12_BIT    ((uint32_t)0x00001000)
#define BT_SWPROF12_POS    12
#define BT_SWPROF11_BIT    ((uint32_t)0x00000800)
#define BT_SWPROF11_POS    11
#define BT_SWPROF10_BIT    ((uint32_t)0x00000400)
#define BT_SWPROF10_POS    10
#define BT_SWPROF9_BIT     ((uint32_t)0x00000200)
#define BT_SWPROF9_POS     9
#define BT_SWPROF8_BIT     ((uint32_t)0x00000100)
#define BT_SWPROF8_POS     8
#define BT_SWPROF7_BIT     ((uint32_t)0x00000080)
#define BT_SWPROF7_POS     7
#define BT_SWPROF6_BIT     ((uint32_t)0x00000040)
#define BT_SWPROF6_POS     6
#define BT_SWPROF5_BIT     ((uint32_t)0x00000020)
#define BT_SWPROF5_POS     5
#define BT_SWPROF4_BIT     ((uint32_t)0x00000010)
#define BT_SWPROF4_POS     4
#define BT_SWPROF3_BIT     ((uint32_t)0x00000008)
#define BT_SWPROF3_POS     3
#define BT_SWPROF2_BIT     ((uint32_t)0x00000004)
#define BT_SWPROF2_POS     2
#define BT_SWPROF1_BIT     ((uint32_t)0x00000002)
#define BT_SWPROF1_POS     1
#define BT_SWPROF0_BIT     ((uint32_t)0x00000001)
#define BT_SWPROF0_POS     0

#define BT_SWPROF31_RST    0x0
#define BT_SWPROF30_RST    0x0
#define BT_SWPROF29_RST    0x0
#define BT_SWPROF28_RST    0x0
#define BT_SWPROF27_RST    0x0
#define BT_SWPROF26_RST    0x0
#define BT_SWPROF25_RST    0x0
#define BT_SWPROF24_RST    0x0
#define BT_SWPROF23_RST    0x0
#define BT_SWPROF22_RST    0x0
#define BT_SWPROF21_RST    0x0
#define BT_SWPROF20_RST    0x0
#define BT_SWPROF19_RST    0x0
#define BT_SWPROF18_RST    0x0
#define BT_SWPROF17_RST    0x0
#define BT_SWPROF16_RST    0x0
#define BT_SWPROF15_RST    0x0
#define BT_SWPROF14_RST    0x0
#define BT_SWPROF13_RST    0x0
#define BT_SWPROF12_RST    0x0
#define BT_SWPROF11_RST    0x0
#define BT_SWPROF10_RST    0x0
#define BT_SWPROF9_RST     0x0
#define BT_SWPROF8_RST     0x0
#define BT_SWPROF7_RST     0x0
#define BT_SWPROF6_RST     0x0
#define BT_SWPROF5_RST     0x0
#define BT_SWPROF4_RST     0x0
#define BT_SWPROF3_RST     0x0
#define BT_SWPROF2_RST     0x0
#define BT_SWPROF1_RST     0x0
#define BT_SWPROF0_RST     0x0

__INLINE void bt_swprofiling_pack(uint8_t swprof31, uint8_t swprof30, uint8_t swprof29, uint8_t swprof28, uint8_t swprof27, uint8_t swprof26, uint8_t swprof25, uint8_t swprof24, uint8_t swprof23, uint8_t swprof22, uint8_t swprof21, uint8_t swprof20, uint8_t swprof19, uint8_t swprof18, uint8_t swprof17, uint8_t swprof16, uint8_t swprof15, uint8_t swprof14, uint8_t swprof13, uint8_t swprof12, uint8_t swprof11, uint8_t swprof10, uint8_t swprof9, uint8_t swprof8, uint8_t swprof7, uint8_t swprof6, uint8_t swprof5, uint8_t swprof4, uint8_t swprof3, uint8_t swprof2, uint8_t swprof1, uint8_t swprof0)
{
    ASSERT_ERR((((uint32_t)swprof31 << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof30 << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof29 << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof28 << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof27 << 27) & ~((uint32_t)0x08000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof26 << 26) & ~((uint32_t)0x04000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof25 << 25) & ~((uint32_t)0x02000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof24 << 24) & ~((uint32_t)0x01000000)) == 0);
    ASSERT_ERR((((uint32_t)swprof23 << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)swprof22 << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)swprof21 << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)swprof20 << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)swprof19 << 19) & ~((uint32_t)0x00080000)) == 0);
    ASSERT_ERR((((uint32_t)swprof18 << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)swprof17 << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)swprof16 << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)swprof15 << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)swprof14 << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)swprof13 << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)swprof12 << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)swprof11 << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)swprof10 << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)swprof9 << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)swprof8 << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)swprof7 << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)swprof6 << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)swprof5 << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)swprof4 << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)swprof3 << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)swprof2 << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)swprof1 << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)swprof0 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, ((uint32_t)swprof31 << 31) | ((uint32_t)swprof30 << 30) | ((uint32_t)swprof29 << 29) | ((uint32_t)swprof28 << 28) | ((uint32_t)swprof27 << 27) | ((uint32_t)swprof26 << 26) | ((uint32_t)swprof25 << 25) | ((uint32_t)swprof24 << 24) | ((uint32_t)swprof23 << 23) | ((uint32_t)swprof22 << 22) | ((uint32_t)swprof21 << 21) | ((uint32_t)swprof20 << 20) | ((uint32_t)swprof19 << 19) | ((uint32_t)swprof18 << 18) | ((uint32_t)swprof17 << 17) | ((uint32_t)swprof16 << 16) | ((uint32_t)swprof15 << 15) | ((uint32_t)swprof14 << 14) | ((uint32_t)swprof13 << 13) | ((uint32_t)swprof12 << 12) | ((uint32_t)swprof11 << 11) | ((uint32_t)swprof10 << 10) | ((uint32_t)swprof9 << 9) | ((uint32_t)swprof8 << 8) | ((uint32_t)swprof7 << 7) | ((uint32_t)swprof6 << 6) | ((uint32_t)swprof5 << 5) | ((uint32_t)swprof4 << 4) | ((uint32_t)swprof3 << 3) | ((uint32_t)swprof2 << 2) | ((uint32_t)swprof1 << 1) | ((uint32_t)swprof0 << 0));
}

__INLINE void bt_swprofiling_unpack(uint8_t *swprof31, uint8_t *swprof30, uint8_t *swprof29, uint8_t *swprof28, uint8_t *swprof27, uint8_t *swprof26, uint8_t *swprof25, uint8_t *swprof24, uint8_t *swprof23, uint8_t *swprof22, uint8_t *swprof21, uint8_t *swprof20, uint8_t *swprof19, uint8_t *swprof18, uint8_t *swprof17, uint8_t *swprof16, uint8_t *swprof15, uint8_t *swprof14, uint8_t *swprof13, uint8_t *swprof12, uint8_t *swprof11, uint8_t *swprof10, uint8_t *swprof9, uint8_t *swprof8, uint8_t *swprof7, uint8_t *swprof6, uint8_t *swprof5, uint8_t *swprof4, uint8_t *swprof3, uint8_t *swprof2, uint8_t *swprof1, uint8_t *swprof0)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);

    *swprof31 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *swprof30 = (localVal & ((uint32_t)0x40000000)) >> 30;
    *swprof29 = (localVal & ((uint32_t)0x20000000)) >> 29;
    *swprof28 = (localVal & ((uint32_t)0x10000000)) >> 28;
    *swprof27 = (localVal & ((uint32_t)0x08000000)) >> 27;
    *swprof26 = (localVal & ((uint32_t)0x04000000)) >> 26;
    *swprof25 = (localVal & ((uint32_t)0x02000000)) >> 25;
    *swprof24 = (localVal & ((uint32_t)0x01000000)) >> 24;
    *swprof23 = (localVal & ((uint32_t)0x00800000)) >> 23;
    *swprof22 = (localVal & ((uint32_t)0x00400000)) >> 22;
    *swprof21 = (localVal & ((uint32_t)0x00200000)) >> 21;
    *swprof20 = (localVal & ((uint32_t)0x00100000)) >> 20;
    *swprof19 = (localVal & ((uint32_t)0x00080000)) >> 19;
    *swprof18 = (localVal & ((uint32_t)0x00040000)) >> 18;
    *swprof17 = (localVal & ((uint32_t)0x00020000)) >> 17;
    *swprof16 = (localVal & ((uint32_t)0x00010000)) >> 16;
    *swprof15 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *swprof14 = (localVal & ((uint32_t)0x00004000)) >> 14;
    *swprof13 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *swprof12 = (localVal & ((uint32_t)0x00001000)) >> 12;
    *swprof11 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *swprof10 = (localVal & ((uint32_t)0x00000400)) >> 10;
    *swprof9 = (localVal & ((uint32_t)0x00000200)) >> 9;
    *swprof8 = (localVal & ((uint32_t)0x00000100)) >> 8;
    *swprof7 = (localVal & ((uint32_t)0x00000080)) >> 7;
    *swprof6 = (localVal & ((uint32_t)0x00000040)) >> 6;
    *swprof5 = (localVal & ((uint32_t)0x00000020)) >> 5;
    *swprof4 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *swprof3 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *swprof2 = (localVal & ((uint32_t)0x00000004)) >> 2;
    *swprof1 = (localVal & ((uint32_t)0x00000002)) >> 1;
    *swprof0 = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_swprofiling_swprof31_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_swprofiling_swprof31_setf(uint8_t swprof31)
{
    ASSERT_ERR((((uint32_t)swprof31 << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)swprof31 << 31));
}

__INLINE uint8_t bt_swprofiling_swprof30_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void bt_swprofiling_swprof30_setf(uint8_t swprof30)
{
    ASSERT_ERR((((uint32_t)swprof30 << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)swprof30 << 30));
}

__INLINE uint8_t bt_swprofiling_swprof29_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void bt_swprofiling_swprof29_setf(uint8_t swprof29)
{
    ASSERT_ERR((((uint32_t)swprof29 << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)swprof29 << 29));
}

__INLINE uint8_t bt_swprofiling_swprof28_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void bt_swprofiling_swprof28_setf(uint8_t swprof28)
{
    ASSERT_ERR((((uint32_t)swprof28 << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)swprof28 << 28));
}

__INLINE uint8_t bt_swprofiling_swprof27_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void bt_swprofiling_swprof27_setf(uint8_t swprof27)
{
    ASSERT_ERR((((uint32_t)swprof27 << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)swprof27 << 27));
}

__INLINE uint8_t bt_swprofiling_swprof26_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void bt_swprofiling_swprof26_setf(uint8_t swprof26)
{
    ASSERT_ERR((((uint32_t)swprof26 << 26) & ~((uint32_t)0x04000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)swprof26 << 26));
}

__INLINE uint8_t bt_swprofiling_swprof25_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void bt_swprofiling_swprof25_setf(uint8_t swprof25)
{
    ASSERT_ERR((((uint32_t)swprof25 << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)swprof25 << 25));
}

__INLINE uint8_t bt_swprofiling_swprof24_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void bt_swprofiling_swprof24_setf(uint8_t swprof24)
{
    ASSERT_ERR((((uint32_t)swprof24 << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)swprof24 << 24));
}

__INLINE uint8_t bt_swprofiling_swprof23_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bt_swprofiling_swprof23_setf(uint8_t swprof23)
{
    ASSERT_ERR((((uint32_t)swprof23 << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)swprof23 << 23));
}

__INLINE uint8_t bt_swprofiling_swprof22_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bt_swprofiling_swprof22_setf(uint8_t swprof22)
{
    ASSERT_ERR((((uint32_t)swprof22 << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)swprof22 << 22));
}

__INLINE uint8_t bt_swprofiling_swprof21_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void bt_swprofiling_swprof21_setf(uint8_t swprof21)
{
    ASSERT_ERR((((uint32_t)swprof21 << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)swprof21 << 21));
}

__INLINE uint8_t bt_swprofiling_swprof20_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void bt_swprofiling_swprof20_setf(uint8_t swprof20)
{
    ASSERT_ERR((((uint32_t)swprof20 << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)swprof20 << 20));
}

__INLINE uint8_t bt_swprofiling_swprof19_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void bt_swprofiling_swprof19_setf(uint8_t swprof19)
{
    ASSERT_ERR((((uint32_t)swprof19 << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)swprof19 << 19));
}

__INLINE uint8_t bt_swprofiling_swprof18_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void bt_swprofiling_swprof18_setf(uint8_t swprof18)
{
    ASSERT_ERR((((uint32_t)swprof18 << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)swprof18 << 18));
}

__INLINE uint8_t bt_swprofiling_swprof17_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void bt_swprofiling_swprof17_setf(uint8_t swprof17)
{
    ASSERT_ERR((((uint32_t)swprof17 << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)swprof17 << 17));
}

__INLINE uint8_t bt_swprofiling_swprof16_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bt_swprofiling_swprof16_setf(uint8_t swprof16)
{
    ASSERT_ERR((((uint32_t)swprof16 << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)swprof16 << 16));
}

__INLINE uint8_t bt_swprofiling_swprof15_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_swprofiling_swprof15_setf(uint8_t swprof15)
{
    ASSERT_ERR((((uint32_t)swprof15 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)swprof15 << 15));
}

__INLINE uint8_t bt_swprofiling_swprof14_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_swprofiling_swprof14_setf(uint8_t swprof14)
{
    ASSERT_ERR((((uint32_t)swprof14 << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)swprof14 << 14));
}

__INLINE uint8_t bt_swprofiling_swprof13_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_swprofiling_swprof13_setf(uint8_t swprof13)
{
    ASSERT_ERR((((uint32_t)swprof13 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)swprof13 << 13));
}

__INLINE uint8_t bt_swprofiling_swprof12_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_swprofiling_swprof12_setf(uint8_t swprof12)
{
    ASSERT_ERR((((uint32_t)swprof12 << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)swprof12 << 12));
}

__INLINE uint8_t bt_swprofiling_swprof11_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void bt_swprofiling_swprof11_setf(uint8_t swprof11)
{
    ASSERT_ERR((((uint32_t)swprof11 << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)swprof11 << 11));
}

__INLINE uint8_t bt_swprofiling_swprof10_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bt_swprofiling_swprof10_setf(uint8_t swprof10)
{
    ASSERT_ERR((((uint32_t)swprof10 << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)swprof10 << 10));
}

__INLINE uint8_t bt_swprofiling_swprof9_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void bt_swprofiling_swprof9_setf(uint8_t swprof9)
{
    ASSERT_ERR((((uint32_t)swprof9 << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)swprof9 << 9));
}

__INLINE uint8_t bt_swprofiling_swprof8_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bt_swprofiling_swprof8_setf(uint8_t swprof8)
{
    ASSERT_ERR((((uint32_t)swprof8 << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)swprof8 << 8));
}

__INLINE uint8_t bt_swprofiling_swprof7_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_swprofiling_swprof7_setf(uint8_t swprof7)
{
    ASSERT_ERR((((uint32_t)swprof7 << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)swprof7 << 7));
}

__INLINE uint8_t bt_swprofiling_swprof6_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bt_swprofiling_swprof6_setf(uint8_t swprof6)
{
    ASSERT_ERR((((uint32_t)swprof6 << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)swprof6 << 6));
}

__INLINE uint8_t bt_swprofiling_swprof5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_swprofiling_swprof5_setf(uint8_t swprof5)
{
    ASSERT_ERR((((uint32_t)swprof5 << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)swprof5 << 5));
}

__INLINE uint8_t bt_swprofiling_swprof4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_swprofiling_swprof4_setf(uint8_t swprof4)
{
    ASSERT_ERR((((uint32_t)swprof4 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)swprof4 << 4));
}

__INLINE uint8_t bt_swprofiling_swprof3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_swprofiling_swprof3_setf(uint8_t swprof3)
{
    ASSERT_ERR((((uint32_t)swprof3 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)swprof3 << 3));
}

__INLINE uint8_t bt_swprofiling_swprof2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_swprofiling_swprof2_setf(uint8_t swprof2)
{
    ASSERT_ERR((((uint32_t)swprof2 << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)swprof2 << 2));
}

__INLINE uint8_t bt_swprofiling_swprof1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_swprofiling_swprof1_setf(uint8_t swprof1)
{
    ASSERT_ERR((((uint32_t)swprof1 << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)swprof1 << 1));
}

__INLINE uint8_t bt_swprofiling_swprof0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_swprofiling_swprof0_setf(uint8_t swprof0)
{
    ASSERT_ERR((((uint32_t)swprof0 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_SWPROFILING_ADDR, (REG_BT_RD(BT_SWPROFILING_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)swprof0 << 0));
}

/**
 * @brief RADIOCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16               SPIPTR   0x0
 *     07               SPICFG   0
 *  05:04              SPIFREQ   0x0
 *     01              SPICOMP   1
 *     00                SPIGO   0
 * </pre>
 */
#define BT_RADIOCNTL0_ADDR   0x50090470
#define BT_RADIOCNTL0_OFFSET 0x00000070
#define BT_RADIOCNTL0_INDEX  0x0000001C
#define BT_RADIOCNTL0_RESET  0x00000002

__INLINE uint32_t bt_radiocntl0_get(void)
{
    return REG_BT_RD(BT_RADIOCNTL0_ADDR);
}

__INLINE void bt_radiocntl0_set(uint32_t value)
{
    REG_BT_WR(BT_RADIOCNTL0_ADDR, value);
}

// field definitions
#define BT_SPIPTR_MASK    ((uint32_t)0x3FFF0000)
#define BT_SPIPTR_LSB     16
#define BT_SPIPTR_WIDTH   ((uint32_t)0x0000000E)
#define BT_SPICFG_BIT     ((uint32_t)0x00000080)
#define BT_SPICFG_POS     7
#define BT_SPIFREQ_MASK   ((uint32_t)0x00000030)
#define BT_SPIFREQ_LSB    4
#define BT_SPIFREQ_WIDTH  ((uint32_t)0x00000002)
#define BT_SPICOMP_BIT    ((uint32_t)0x00000002)
#define BT_SPICOMP_POS    1
#define BT_SPIGO_BIT      ((uint32_t)0x00000001)
#define BT_SPIGO_POS      0

#define BT_SPIPTR_RST     0x0
#define BT_SPICFG_RST     0x0
#define BT_SPIFREQ_RST    0x0
#define BT_SPICOMP_RST    0x1
#define BT_SPIGO_RST      0x0

__INLINE void bt_radiocntl0_pack(uint16_t spiptr, uint8_t spicfg, uint8_t spifreq, uint8_t spigo)
{
    ASSERT_ERR((((uint32_t)spiptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)spicfg << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)spifreq << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)spigo << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_RADIOCNTL0_ADDR, ((uint32_t)spiptr << 16) | ((uint32_t)spicfg << 7) | ((uint32_t)spifreq << 4) | ((uint32_t)spigo << 0));
}

__INLINE void bt_radiocntl0_unpack(uint16_t *spiptr, uint8_t *spicfg, uint8_t *spifreq, uint8_t *spicomp, uint8_t *spigo)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL0_ADDR);

    *spiptr = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *spicfg = (localVal & ((uint32_t)0x00000080)) >> 7;
    *spifreq = (localVal & ((uint32_t)0x00000030)) >> 4;
    *spicomp = (localVal & ((uint32_t)0x00000002)) >> 1;
    *spigo = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t bt_radiocntl0_spiptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_radiocntl0_spiptr_setf(uint16_t spiptr)
{
    ASSERT_ERR((((uint32_t)spiptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_RADIOCNTL0_ADDR, (REG_BT_RD(BT_RADIOCNTL0_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)spiptr << 16));
}

__INLINE uint8_t bt_radiocntl0_spicfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_radiocntl0_spicfg_setf(uint8_t spicfg)
{
    ASSERT_ERR((((uint32_t)spicfg << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_RADIOCNTL0_ADDR, (REG_BT_RD(BT_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)spicfg << 7));
}

__INLINE uint8_t bt_radiocntl0_spifreq_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void bt_radiocntl0_spifreq_setf(uint8_t spifreq)
{
    ASSERT_ERR((((uint32_t)spifreq << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_BT_WR(BT_RADIOCNTL0_ADDR, (REG_BT_RD(BT_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)spifreq << 4));
}

__INLINE uint8_t bt_radiocntl0_spicomp_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t bt_radiocntl0_spigo_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_radiocntl0_spigo_setf(uint8_t spigo)
{
    ASSERT_ERR((((uint32_t)spigo << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_RADIOCNTL0_ADDR, (REG_BT_RD(BT_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)spigo << 0));
}

/**
 * @brief RADIOCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31          FORCEAGC_EN   0
 *     30              FORCEIQ   0
 *     29               RXDNSL   0
 *     28               TXDNSL   0
 *  27:16      FORCEAGC_LENGTH   0x0
 *     15      SYNC_PULSE_MODE   0
 *     14       SYNC_PULSE_SRC   0
 *     13            DPCORR_EN   0
 *     12           JEF_SELECT   0
 *  09:04               XRFSEL   0x0
 *  03:00           SUBVERSION   0x0
 * </pre>
 */
#define BT_RADIOCNTL1_ADDR   0x50090474
#define BT_RADIOCNTL1_OFFSET 0x00000074
#define BT_RADIOCNTL1_INDEX  0x0000001D
#define BT_RADIOCNTL1_RESET  0x00000000

__INLINE uint32_t bt_radiocntl1_get(void)
{
    return REG_BT_RD(BT_RADIOCNTL1_ADDR);
}

__INLINE void bt_radiocntl1_set(uint32_t value)
{
    REG_BT_WR(BT_RADIOCNTL1_ADDR, value);
}

// field definitions
#define BT_FORCEAGC_EN_BIT        ((uint32_t)0x80000000)
#define BT_FORCEAGC_EN_POS        31
#define BT_FORCEIQ_BIT            ((uint32_t)0x40000000)
#define BT_FORCEIQ_POS            30
#define BT_RXDNSL_BIT             ((uint32_t)0x20000000)
#define BT_RXDNSL_POS             29
#define BT_TXDNSL_BIT             ((uint32_t)0x10000000)
#define BT_TXDNSL_POS             28
#define BT_FORCEAGC_LENGTH_MASK   ((uint32_t)0x0FFF0000)
#define BT_FORCEAGC_LENGTH_LSB    16
#define BT_FORCEAGC_LENGTH_WIDTH  ((uint32_t)0x0000000C)
#define BT_SYNC_PULSE_MODE_BIT    ((uint32_t)0x00008000)
#define BT_SYNC_PULSE_MODE_POS    15
#define BT_SYNC_PULSE_SRC_BIT     ((uint32_t)0x00004000)
#define BT_SYNC_PULSE_SRC_POS     14
#define BT_DPCORR_EN_BIT          ((uint32_t)0x00002000)
#define BT_DPCORR_EN_POS          13
#define BT_JEF_SELECT_BIT         ((uint32_t)0x00001000)
#define BT_JEF_SELECT_POS         12
#define BT_XRFSEL_MASK            ((uint32_t)0x000003F0)
#define BT_XRFSEL_LSB             4
#define BT_XRFSEL_WIDTH           ((uint32_t)0x00000006)
#define BT_SUBVERSION_MASK        ((uint32_t)0x0000000F)
#define BT_SUBVERSION_LSB         0
#define BT_SUBVERSION_WIDTH       ((uint32_t)0x00000004)

#define BT_FORCEAGC_EN_RST        0x0
#define BT_FORCEIQ_RST            0x0
#define BT_RXDNSL_RST             0x0
#define BT_TXDNSL_RST             0x0
#define BT_FORCEAGC_LENGTH_RST    0x0
#define BT_SYNC_PULSE_MODE_RST    0x0
#define BT_SYNC_PULSE_SRC_RST     0x0
#define BT_DPCORR_EN_RST          0x0
#define BT_JEF_SELECT_RST         0x0
#define BT_XRFSEL_RST             0x0
#define BT_SUBVERSION_RST         0x0

__INLINE void bt_radiocntl1_pack(uint8_t forceagcen, uint8_t forceiq, uint8_t rxdnsl, uint8_t txdnsl, uint16_t forceagclength, uint8_t syncpulsemode, uint8_t syncpulsesrc, uint8_t dpcorren, uint8_t jefselect, uint8_t xrfsel, uint8_t subversion)
{
    ASSERT_ERR((((uint32_t)forceagcen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)forceiq << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)rxdnsl << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)txdnsl << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)forceagclength << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)syncpulsemode << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)syncpulsesrc << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)dpcorren << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)jefselect << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)xrfsel << 4) & ~((uint32_t)0x000003F0)) == 0);
    ASSERT_ERR((((uint32_t)subversion << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, ((uint32_t)forceagcen << 31) | ((uint32_t)forceiq << 30) | ((uint32_t)rxdnsl << 29) | ((uint32_t)txdnsl << 28) | ((uint32_t)forceagclength << 16) | ((uint32_t)syncpulsemode << 15) | ((uint32_t)syncpulsesrc << 14) | ((uint32_t)dpcorren << 13) | ((uint32_t)jefselect << 12) | ((uint32_t)xrfsel << 4) | ((uint32_t)subversion << 0));
}

__INLINE void bt_radiocntl1_unpack(uint8_t *forceagcen, uint8_t *forceiq, uint8_t *rxdnsl, uint8_t *txdnsl, uint16_t *forceagclength, uint8_t *syncpulsemode, uint8_t *syncpulsesrc, uint8_t *dpcorren, uint8_t *jefselect, uint8_t *xrfsel, uint8_t *subversion)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);

    *forceagcen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *forceiq = (localVal & ((uint32_t)0x40000000)) >> 30;
    *rxdnsl = (localVal & ((uint32_t)0x20000000)) >> 29;
    *txdnsl = (localVal & ((uint32_t)0x10000000)) >> 28;
    *forceagclength = (localVal & ((uint32_t)0x0FFF0000)) >> 16;
    *syncpulsemode = (localVal & ((uint32_t)0x00008000)) >> 15;
    *syncpulsesrc = (localVal & ((uint32_t)0x00004000)) >> 14;
    *dpcorren = (localVal & ((uint32_t)0x00002000)) >> 13;
    *jefselect = (localVal & ((uint32_t)0x00001000)) >> 12;
    *xrfsel = (localVal & ((uint32_t)0x000003F0)) >> 4;
    *subversion = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_radiocntl1_forceagc_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_radiocntl1_forceagc_en_setf(uint8_t forceagcen)
{
    ASSERT_ERR((((uint32_t)forceagcen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)forceagcen << 31));
}

__INLINE uint8_t bt_radiocntl1_forceiq_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void bt_radiocntl1_forceiq_setf(uint8_t forceiq)
{
    ASSERT_ERR((((uint32_t)forceiq << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)forceiq << 30));
}

__INLINE uint8_t bt_radiocntl1_rxdnsl_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void bt_radiocntl1_rxdnsl_setf(uint8_t rxdnsl)
{
    ASSERT_ERR((((uint32_t)rxdnsl << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)rxdnsl << 29));
}

__INLINE uint8_t bt_radiocntl1_txdnsl_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void bt_radiocntl1_txdnsl_setf(uint8_t txdnsl)
{
    ASSERT_ERR((((uint32_t)txdnsl << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)txdnsl << 28));
}

__INLINE uint16_t bt_radiocntl1_forceagc_length_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void bt_radiocntl1_forceagc_length_setf(uint16_t forceagclength)
{
    ASSERT_ERR((((uint32_t)forceagclength << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)forceagclength << 16));
}

__INLINE uint8_t bt_radiocntl1_sync_pulse_mode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_radiocntl1_sync_pulse_mode_setf(uint8_t syncpulsemode)
{
    ASSERT_ERR((((uint32_t)syncpulsemode << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)syncpulsemode << 15));
}

__INLINE uint8_t bt_radiocntl1_sync_pulse_src_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_radiocntl1_sync_pulse_src_setf(uint8_t syncpulsesrc)
{
    ASSERT_ERR((((uint32_t)syncpulsesrc << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)syncpulsesrc << 14));
}

__INLINE uint8_t bt_radiocntl1_dpcorr_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_radiocntl1_dpcorr_en_setf(uint8_t dpcorren)
{
    ASSERT_ERR((((uint32_t)dpcorren << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)dpcorren << 13));
}

__INLINE uint8_t bt_radiocntl1_jef_select_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_radiocntl1_jef_select_setf(uint8_t jefselect)
{
    ASSERT_ERR((((uint32_t)jefselect << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)jefselect << 12));
}

__INLINE uint8_t bt_radiocntl1_xrfsel_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000003F0)) >> 4);
}

__INLINE void bt_radiocntl1_xrfsel_setf(uint8_t xrfsel)
{
    ASSERT_ERR((((uint32_t)xrfsel << 4) & ~((uint32_t)0x000003F0)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x000003F0)) | ((uint32_t)xrfsel << 4));
}

__INLINE uint8_t bt_radiocntl1_subversion_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_radiocntl1_subversion_setf(uint8_t subversion)
{
    ASSERT_ERR((((uint32_t)subversion << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_RADIOCNTL1_ADDR, (REG_BT_RD(BT_RADIOCNTL1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)subversion << 0));
}

/**
 * @brief RADIOCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24   TRAILER_GATING_VAL   0x4
 *  19:16              SYNCERR   0x0
 *  13:00        FREQTABLE_PTR   0x40
 * </pre>
 */
#define BT_RADIOCNTL2_ADDR   0x50090478
#define BT_RADIOCNTL2_OFFSET 0x00000078
#define BT_RADIOCNTL2_INDEX  0x0000001E
#define BT_RADIOCNTL2_RESET  0x04000040

__INLINE uint32_t bt_radiocntl2_get(void)
{
    return REG_BT_RD(BT_RADIOCNTL2_ADDR);
}

__INLINE void bt_radiocntl2_set(uint32_t value)
{
    REG_BT_WR(BT_RADIOCNTL2_ADDR, value);
}

// field definitions
#define BT_TRAILER_GATING_VAL_MASK   ((uint32_t)0x07000000)
#define BT_TRAILER_GATING_VAL_LSB    24
#define BT_TRAILER_GATING_VAL_WIDTH  ((uint32_t)0x00000003)
#define BT_SYNCERR_MASK              ((uint32_t)0x000F0000)
#define BT_SYNCERR_LSB               16
#define BT_SYNCERR_WIDTH             ((uint32_t)0x00000004)
#define BT_FREQTABLE_PTR_MASK        ((uint32_t)0x00003FFF)
#define BT_FREQTABLE_PTR_LSB         0
#define BT_FREQTABLE_PTR_WIDTH       ((uint32_t)0x0000000E)

#define BT_TRAILER_GATING_VAL_RST    0x4
#define BT_SYNCERR_RST               0x0
#define BT_FREQTABLE_PTR_RST         0x40

__INLINE void bt_radiocntl2_pack(uint8_t trailergatingval, uint8_t syncerr, uint16_t freqtableptr)
{
    ASSERT_ERR((((uint32_t)trailergatingval << 24) & ~((uint32_t)0x07000000)) == 0);
    ASSERT_ERR((((uint32_t)syncerr << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)freqtableptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_RADIOCNTL2_ADDR, ((uint32_t)trailergatingval << 24) | ((uint32_t)syncerr << 16) | ((uint32_t)freqtableptr << 0));
}

__INLINE void bt_radiocntl2_unpack(uint8_t *trailergatingval, uint8_t *syncerr, uint16_t *freqtableptr)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL2_ADDR);

    *trailergatingval = (localVal & ((uint32_t)0x07000000)) >> 24;
    *syncerr = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *freqtableptr = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t bt_radiocntl2_trailer_gating_val_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void bt_radiocntl2_trailer_gating_val_setf(uint8_t trailergatingval)
{
    ASSERT_ERR((((uint32_t)trailergatingval << 24) & ~((uint32_t)0x07000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL2_ADDR, (REG_BT_RD(BT_RADIOCNTL2_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)trailergatingval << 24));
}

__INLINE uint8_t bt_radiocntl2_syncerr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_radiocntl2_syncerr_setf(uint8_t syncerr)
{
    ASSERT_ERR((((uint32_t)syncerr << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_RADIOCNTL2_ADDR, (REG_BT_RD(BT_RADIOCNTL2_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)syncerr << 16));
}

__INLINE uint16_t bt_radiocntl2_freqtable_ptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_radiocntl2_freqtable_ptr_setf(uint16_t freqtableptr)
{
    ASSERT_ERR((((uint32_t)freqtableptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_RADIOCNTL2_ADDR, (REG_BT_RD(BT_RADIOCNTL2_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)freqtableptr << 0));
}

/**
 * @brief RADIOCNTL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:28           RXRATE2CFG   0x3
 *  27:26           RXRATE1CFG   0x2
 *  25:24           RXRATE0CFG   0x0
 *  22:20         GETRSSIDELAY   0x4
 *     19         RX_SERPAR_IF   0
 *     18       RXSYNC_ROUTING   0
 *  17:16          RXVALID_BEH   0x0
 *  13:12           TXRATE2CFG   0x3
 *  11:10           TXRATE1CFG   0x2
 *  09:08           TXRATE0CFG   0x0
 *     02         TX_SERPAR_IF   0
 *  01:00          TXVALID_BEH   0x0
 * </pre>
 */
#define BT_RADIOCNTL3_ADDR   0x5009047C
#define BT_RADIOCNTL3_OFFSET 0x0000007C
#define BT_RADIOCNTL3_INDEX  0x0000001F
#define BT_RADIOCNTL3_RESET  0x38403800

__INLINE uint32_t bt_radiocntl3_get(void)
{
    return REG_BT_RD(BT_RADIOCNTL3_ADDR);
}

__INLINE void bt_radiocntl3_set(uint32_t value)
{
    REG_BT_WR(BT_RADIOCNTL3_ADDR, value);
}

// field definitions
#define BT_RXRATE2CFG_MASK       ((uint32_t)0x30000000)
#define BT_RXRATE2CFG_LSB        28
#define BT_RXRATE2CFG_WIDTH      ((uint32_t)0x00000002)
#define BT_RXRATE1CFG_MASK       ((uint32_t)0x0C000000)
#define BT_RXRATE1CFG_LSB        26
#define BT_RXRATE1CFG_WIDTH      ((uint32_t)0x00000002)
#define BT_RXRATE0CFG_MASK       ((uint32_t)0x03000000)
#define BT_RXRATE0CFG_LSB        24
#define BT_RXRATE0CFG_WIDTH      ((uint32_t)0x00000002)
#define BT_GETRSSIDELAY_MASK     ((uint32_t)0x00700000)
#define BT_GETRSSIDELAY_LSB      20
#define BT_GETRSSIDELAY_WIDTH    ((uint32_t)0x00000003)
#define BT_RX_SERPAR_IF_BIT      ((uint32_t)0x00080000)
#define BT_RX_SERPAR_IF_POS      19
#define BT_RXSYNC_ROUTING_BIT    ((uint32_t)0x00040000)
#define BT_RXSYNC_ROUTING_POS    18
#define BT_RXVALID_BEH_MASK      ((uint32_t)0x00030000)
#define BT_RXVALID_BEH_LSB       16
#define BT_RXVALID_BEH_WIDTH     ((uint32_t)0x00000002)
#define BT_TXRATE2CFG_MASK       ((uint32_t)0x00003000)
#define BT_TXRATE2CFG_LSB        12
#define BT_TXRATE2CFG_WIDTH      ((uint32_t)0x00000002)
#define BT_TXRATE1CFG_MASK       ((uint32_t)0x00000C00)
#define BT_TXRATE1CFG_LSB        10
#define BT_TXRATE1CFG_WIDTH      ((uint32_t)0x00000002)
#define BT_TXRATE0CFG_MASK       ((uint32_t)0x00000300)
#define BT_TXRATE0CFG_LSB        8
#define BT_TXRATE0CFG_WIDTH      ((uint32_t)0x00000002)
#define BT_TX_SERPAR_IF_BIT      ((uint32_t)0x00000004)
#define BT_TX_SERPAR_IF_POS      2
#define BT_TXVALID_BEH_MASK      ((uint32_t)0x00000003)
#define BT_TXVALID_BEH_LSB       0
#define BT_TXVALID_BEH_WIDTH     ((uint32_t)0x00000002)

#define BT_RXRATE2CFG_RST        0x3
#define BT_RXRATE1CFG_RST        0x2
#define BT_RXRATE0CFG_RST        0x0
#define BT_GETRSSIDELAY_RST      0x4
#define BT_RX_SERPAR_IF_RST      0x0
#define BT_RXSYNC_ROUTING_RST    0x0
#define BT_RXVALID_BEH_RST       0x0
#define BT_TXRATE2CFG_RST        0x3
#define BT_TXRATE1CFG_RST        0x2
#define BT_TXRATE0CFG_RST        0x0
#define BT_TX_SERPAR_IF_RST      0x0
#define BT_TXVALID_BEH_RST       0x0

__INLINE void bt_radiocntl3_pack(uint8_t rxrate2cfg, uint8_t rxrate1cfg, uint8_t rxrate0cfg, uint8_t getrssidelay, uint8_t rxserparif, uint8_t rxsyncrouting, uint8_t rxvalidbeh, uint8_t txrate2cfg, uint8_t txrate1cfg, uint8_t txrate0cfg, uint8_t txserparif, uint8_t txvalidbeh)
{
    ASSERT_ERR((((uint32_t)rxrate2cfg << 28) & ~((uint32_t)0x30000000)) == 0);
    ASSERT_ERR((((uint32_t)rxrate1cfg << 26) & ~((uint32_t)0x0C000000)) == 0);
    ASSERT_ERR((((uint32_t)rxrate0cfg << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)getrssidelay << 20) & ~((uint32_t)0x00700000)) == 0);
    ASSERT_ERR((((uint32_t)rxserparif << 19) & ~((uint32_t)0x00080000)) == 0);
    ASSERT_ERR((((uint32_t)rxsyncrouting << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)rxvalidbeh << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)txrate2cfg << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)txrate1cfg << 10) & ~((uint32_t)0x00000C00)) == 0);
    ASSERT_ERR((((uint32_t)txrate0cfg << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)txserparif << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)txvalidbeh << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, ((uint32_t)rxrate2cfg << 28) | ((uint32_t)rxrate1cfg << 26) | ((uint32_t)rxrate0cfg << 24) | ((uint32_t)getrssidelay << 20) | ((uint32_t)rxserparif << 19) | ((uint32_t)rxsyncrouting << 18) | ((uint32_t)rxvalidbeh << 16) | ((uint32_t)txrate2cfg << 12) | ((uint32_t)txrate1cfg << 10) | ((uint32_t)txrate0cfg << 8) | ((uint32_t)txserparif << 2) | ((uint32_t)txvalidbeh << 0));
}

__INLINE void bt_radiocntl3_unpack(uint8_t *rxrate2cfg, uint8_t *rxrate1cfg, uint8_t *rxrate0cfg, uint8_t *getrssidelay, uint8_t *rxserparif, uint8_t *rxsyncrouting, uint8_t *rxvalidbeh, uint8_t *txrate2cfg, uint8_t *txrate1cfg, uint8_t *txrate0cfg, uint8_t *txserparif, uint8_t *txvalidbeh)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);

    *rxrate2cfg = (localVal & ((uint32_t)0x30000000)) >> 28;
    *rxrate1cfg = (localVal & ((uint32_t)0x0C000000)) >> 26;
    *rxrate0cfg = (localVal & ((uint32_t)0x03000000)) >> 24;
    *getrssidelay = (localVal & ((uint32_t)0x00700000)) >> 20;
    *rxserparif = (localVal & ((uint32_t)0x00080000)) >> 19;
    *rxsyncrouting = (localVal & ((uint32_t)0x00040000)) >> 18;
    *rxvalidbeh = (localVal & ((uint32_t)0x00030000)) >> 16;
    *txrate2cfg = (localVal & ((uint32_t)0x00003000)) >> 12;
    *txrate1cfg = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *txrate0cfg = (localVal & ((uint32_t)0x00000300)) >> 8;
    *txserparif = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txvalidbeh = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t bt_radiocntl3_rxrate2cfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void bt_radiocntl3_rxrate2cfg_setf(uint8_t rxrate2cfg)
{
    ASSERT_ERR((((uint32_t)rxrate2cfg << 28) & ~((uint32_t)0x30000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)rxrate2cfg << 28));
}

__INLINE uint8_t bt_radiocntl3_rxrate1cfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x0C000000)) >> 26);
}

__INLINE void bt_radiocntl3_rxrate1cfg_setf(uint8_t rxrate1cfg)
{
    ASSERT_ERR((((uint32_t)rxrate1cfg << 26) & ~((uint32_t)0x0C000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x0C000000)) | ((uint32_t)rxrate1cfg << 26));
}

__INLINE uint8_t bt_radiocntl3_rxrate0cfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void bt_radiocntl3_rxrate0cfg_setf(uint8_t rxrate0cfg)
{
    ASSERT_ERR((((uint32_t)rxrate0cfg << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rxrate0cfg << 24));
}

__INLINE uint8_t bt_radiocntl3_getrssidelay_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00700000)) >> 20);
}

__INLINE void bt_radiocntl3_getrssidelay_setf(uint8_t getrssidelay)
{
    ASSERT_ERR((((uint32_t)getrssidelay << 20) & ~((uint32_t)0x00700000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00700000)) | ((uint32_t)getrssidelay << 20));
}

__INLINE uint8_t bt_radiocntl3_rx_serpar_if_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void bt_radiocntl3_rx_serpar_if_setf(uint8_t rxserparif)
{
    ASSERT_ERR((((uint32_t)rxserparif << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)rxserparif << 19));
}

__INLINE uint8_t bt_radiocntl3_rxsync_routing_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void bt_radiocntl3_rxsync_routing_setf(uint8_t rxsyncrouting)
{
    ASSERT_ERR((((uint32_t)rxsyncrouting << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)rxsyncrouting << 18));
}

__INLINE uint8_t bt_radiocntl3_rxvalid_beh_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_radiocntl3_rxvalid_beh_setf(uint8_t rxvalidbeh)
{
    ASSERT_ERR((((uint32_t)rxvalidbeh << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)rxvalidbeh << 16));
}

__INLINE uint8_t bt_radiocntl3_txrate2cfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void bt_radiocntl3_txrate2cfg_setf(uint8_t txrate2cfg)
{
    ASSERT_ERR((((uint32_t)txrate2cfg << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)txrate2cfg << 12));
}

__INLINE uint8_t bt_radiocntl3_txrate1cfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void bt_radiocntl3_txrate1cfg_setf(uint8_t txrate1cfg)
{
    ASSERT_ERR((((uint32_t)txrate1cfg << 10) & ~((uint32_t)0x00000C00)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)txrate1cfg << 10));
}

__INLINE uint8_t bt_radiocntl3_txrate0cfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void bt_radiocntl3_txrate0cfg_setf(uint8_t txrate0cfg)
{
    ASSERT_ERR((((uint32_t)txrate0cfg << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)txrate0cfg << 8));
}

__INLINE uint8_t bt_radiocntl3_tx_serpar_if_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_radiocntl3_tx_serpar_if_setf(uint8_t txserparif)
{
    ASSERT_ERR((((uint32_t)txserparif << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)txserparif << 2));
}

__INLINE uint8_t bt_radiocntl3_txvalid_beh_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void bt_radiocntl3_txvalid_beh_setf(uint8_t txvalidbeh)
{
    ASSERT_ERR((((uint32_t)txvalidbeh << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BT_WR(BT_RADIOCNTL3_ADDR, (REG_BT_RD(BT_RADIOCNTL3_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)txvalidbeh << 0));
}

/**
 * @brief RADIOPWRUPDN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16            RXPWRUPCT   0xD2
 *  14:08            TXPWRDNCT   0x3
 *  07:00            TXPWRUPCT   0xD2
 * </pre>
 */
#define BT_RADIOPWRUPDN_ADDR   0x5009048C
#define BT_RADIOPWRUPDN_OFFSET 0x0000008C
#define BT_RADIOPWRUPDN_INDEX  0x00000023
#define BT_RADIOPWRUPDN_RESET  0x00D203D2

__INLINE uint32_t bt_radiopwrupdn_get(void)
{
    return REG_BT_RD(BT_RADIOPWRUPDN_ADDR);
}

__INLINE void bt_radiopwrupdn_set(uint32_t value)
{
    REG_BT_WR(BT_RADIOPWRUPDN_ADDR, value);
}

// field definitions
#define BT_RXPWRUPCT_MASK   ((uint32_t)0x00FF0000)
#define BT_RXPWRUPCT_LSB    16
#define BT_RXPWRUPCT_WIDTH  ((uint32_t)0x00000008)
#define BT_TXPWRDNCT_MASK   ((uint32_t)0x00007F00)
#define BT_TXPWRDNCT_LSB    8
#define BT_TXPWRDNCT_WIDTH  ((uint32_t)0x00000007)
#define BT_TXPWRUPCT_MASK   ((uint32_t)0x000000FF)
#define BT_TXPWRUPCT_LSB    0
#define BT_TXPWRUPCT_WIDTH  ((uint32_t)0x00000008)

#define BT_RXPWRUPCT_RST    0xD2
#define BT_TXPWRDNCT_RST    0x3
#define BT_TXPWRUPCT_RST    0xD2

__INLINE void bt_radiopwrupdn_pack(uint8_t rxpwrupct, uint8_t txpwrdnct, uint8_t txpwrupct)
{
    ASSERT_ERR((((uint32_t)rxpwrupct << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)txpwrdnct << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpwrupct << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_RADIOPWRUPDN_ADDR, ((uint32_t)rxpwrupct << 16) | ((uint32_t)txpwrdnct << 8) | ((uint32_t)txpwrupct << 0));
}

__INLINE void bt_radiopwrupdn_unpack(uint8_t *rxpwrupct, uint8_t *txpwrdnct, uint8_t *txpwrupct)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOPWRUPDN_ADDR);

    *rxpwrupct = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txpwrdnct = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpwrupct = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bt_radiopwrupdn_rxpwrupct_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOPWRUPDN_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bt_radiopwrupdn_rxpwrupct_setf(uint8_t rxpwrupct)
{
    ASSERT_ERR((((uint32_t)rxpwrupct << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(BT_RADIOPWRUPDN_ADDR, (REG_BT_RD(BT_RADIOPWRUPDN_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxpwrupct << 16));
}

__INLINE uint8_t bt_radiopwrupdn_txpwrdnct_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOPWRUPDN_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void bt_radiopwrupdn_txpwrdnct_setf(uint8_t txpwrdnct)
{
    ASSERT_ERR((((uint32_t)txpwrdnct << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BT_WR(BT_RADIOPWRUPDN_ADDR, (REG_BT_RD(BT_RADIOPWRUPDN_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)txpwrdnct << 8));
}

__INLINE uint8_t bt_radiopwrupdn_txpwrupct_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOPWRUPDN_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bt_radiopwrupdn_txpwrupct_setf(uint8_t txpwrupct)
{
    ASSERT_ERR((((uint32_t)txpwrupct << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_RADIOPWRUPDN_ADDR, (REG_BT_RD(BT_RADIOPWRUPDN_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txpwrupct << 0));
}

/**
 * @brief RADIOTXRXTIM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        SYNC_POSITION   0x0
 *  14:08            RXPATHDLY   0x0
 *  06:00            TXPATHDLY   0x0
 * </pre>
 */
#define BT_RADIOTXRXTIM_ADDR   0x50090490
#define BT_RADIOTXRXTIM_OFFSET 0x00000090
#define BT_RADIOTXRXTIM_INDEX  0x00000024
#define BT_RADIOTXRXTIM_RESET  0x00000000

__INLINE uint32_t bt_radiotxrxtim_get(void)
{
    return REG_BT_RD(BT_RADIOTXRXTIM_ADDR);
}

__INLINE void bt_radiotxrxtim_set(uint32_t value)
{
    REG_BT_WR(BT_RADIOTXRXTIM_ADDR, value);
}

// field definitions
#define BT_SYNC_POSITION_MASK   ((uint32_t)0xFF000000)
#define BT_SYNC_POSITION_LSB    24
#define BT_SYNC_POSITION_WIDTH  ((uint32_t)0x00000008)
#define BT_RXPATHDLY_MASK       ((uint32_t)0x00007F00)
#define BT_RXPATHDLY_LSB        8
#define BT_RXPATHDLY_WIDTH      ((uint32_t)0x00000007)
#define BT_TXPATHDLY_MASK       ((uint32_t)0x0000007F)
#define BT_TXPATHDLY_LSB        0
#define BT_TXPATHDLY_WIDTH      ((uint32_t)0x00000007)

#define BT_SYNC_POSITION_RST    0x0
#define BT_RXPATHDLY_RST        0x0
#define BT_TXPATHDLY_RST        0x0

__INLINE void bt_radiotxrxtim_pack(uint8_t syncposition, uint8_t rxpathdly, uint8_t txpathdly)
{
    ASSERT_ERR((((uint32_t)syncposition << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rxpathdly << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpathdly << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_RADIOTXRXTIM_ADDR, ((uint32_t)syncposition << 24) | ((uint32_t)rxpathdly << 8) | ((uint32_t)txpathdly << 0));
}

__INLINE void bt_radiotxrxtim_unpack(uint8_t *syncposition, uint8_t *rxpathdly, uint8_t *txpathdly)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOTXRXTIM_ADDR);

    *syncposition = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rxpathdly = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpathdly = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t bt_radiotxrxtim_sync_position_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOTXRXTIM_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bt_radiotxrxtim_sync_position_setf(uint8_t syncposition)
{
    ASSERT_ERR((((uint32_t)syncposition << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_BT_WR(BT_RADIOTXRXTIM_ADDR, (REG_BT_RD(BT_RADIOTXRXTIM_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)syncposition << 24));
}

__INLINE uint8_t bt_radiotxrxtim_rxpathdly_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOTXRXTIM_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void bt_radiotxrxtim_rxpathdly_setf(uint8_t rxpathdly)
{
    ASSERT_ERR((((uint32_t)rxpathdly << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BT_WR(BT_RADIOTXRXTIM_ADDR, (REG_BT_RD(BT_RADIOTXRXTIM_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)rxpathdly << 8));
}

__INLINE uint8_t bt_radiotxrxtim_txpathdly_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RADIOTXRXTIM_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void bt_radiotxrxtim_txpathdly_setf(uint8_t txpathdly)
{
    ASSERT_ERR((((uint32_t)txpathdly << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_RADIOTXRXTIM_ADDR, (REG_BT_RD(BT_RADIOTXRXTIM_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)txpathdly << 0));
}

/**
 * @brief SPIPTRCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16             TXOFFPTR   0x0
 *  13:00              TXONPTR   0x0
 * </pre>
 */
#define BT_SPIPTRCNTL0_ADDR   0x500904A0
#define BT_SPIPTRCNTL0_OFFSET 0x000000A0
#define BT_SPIPTRCNTL0_INDEX  0x00000028
#define BT_SPIPTRCNTL0_RESET  0x00000000

__INLINE uint32_t bt_spiptrcntl0_get(void)
{
    return REG_BT_RD(BT_SPIPTRCNTL0_ADDR);
}

__INLINE void bt_spiptrcntl0_set(uint32_t value)
{
    REG_BT_WR(BT_SPIPTRCNTL0_ADDR, value);
}

// field definitions
#define BT_TXOFFPTR_MASK   ((uint32_t)0x3FFF0000)
#define BT_TXOFFPTR_LSB    16
#define BT_TXOFFPTR_WIDTH  ((uint32_t)0x0000000E)
#define BT_TXONPTR_MASK    ((uint32_t)0x00003FFF)
#define BT_TXONPTR_LSB     0
#define BT_TXONPTR_WIDTH   ((uint32_t)0x0000000E)

#define BT_TXOFFPTR_RST    0x0
#define BT_TXONPTR_RST     0x0

__INLINE void bt_spiptrcntl0_pack(uint16_t txoffptr, uint16_t txonptr)
{
    ASSERT_ERR((((uint32_t)txoffptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)txonptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL0_ADDR, ((uint32_t)txoffptr << 16) | ((uint32_t)txonptr << 0));
}

__INLINE void bt_spiptrcntl0_unpack(uint16_t *txoffptr, uint16_t *txonptr)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL0_ADDR);

    *txoffptr = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *txonptr = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_spiptrcntl0_txoffptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_spiptrcntl0_txoffptr_setf(uint16_t txoffptr)
{
    ASSERT_ERR((((uint32_t)txoffptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL0_ADDR, (REG_BT_RD(BT_SPIPTRCNTL0_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)txoffptr << 16));
}

__INLINE uint16_t bt_spiptrcntl0_txonptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_spiptrcntl0_txonptr_setf(uint16_t txonptr)
{
    ASSERT_ERR((((uint32_t)txonptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL0_ADDR, (REG_BT_RD(BT_SPIPTRCNTL0_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)txonptr << 0));
}

/**
 * @brief SPIPTRCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16             RXOFFPTR   0x0
 *  13:00              RXONPTR   0x0
 * </pre>
 */
#define BT_SPIPTRCNTL1_ADDR   0x500904A4
#define BT_SPIPTRCNTL1_OFFSET 0x000000A4
#define BT_SPIPTRCNTL1_INDEX  0x00000029
#define BT_SPIPTRCNTL1_RESET  0x00000000

__INLINE uint32_t bt_spiptrcntl1_get(void)
{
    return REG_BT_RD(BT_SPIPTRCNTL1_ADDR);
}

__INLINE void bt_spiptrcntl1_set(uint32_t value)
{
    REG_BT_WR(BT_SPIPTRCNTL1_ADDR, value);
}

// field definitions
#define BT_RXOFFPTR_MASK   ((uint32_t)0x3FFF0000)
#define BT_RXOFFPTR_LSB    16
#define BT_RXOFFPTR_WIDTH  ((uint32_t)0x0000000E)
#define BT_RXONPTR_MASK    ((uint32_t)0x00003FFF)
#define BT_RXONPTR_LSB     0
#define BT_RXONPTR_WIDTH   ((uint32_t)0x0000000E)

#define BT_RXOFFPTR_RST    0x0
#define BT_RXONPTR_RST     0x0

__INLINE void bt_spiptrcntl1_pack(uint16_t rxoffptr, uint16_t rxonptr)
{
    ASSERT_ERR((((uint32_t)rxoffptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)rxonptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL1_ADDR, ((uint32_t)rxoffptr << 16) | ((uint32_t)rxonptr << 0));
}

__INLINE void bt_spiptrcntl1_unpack(uint16_t *rxoffptr, uint16_t *rxonptr)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL1_ADDR);

    *rxoffptr = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *rxonptr = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_spiptrcntl1_rxoffptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_spiptrcntl1_rxoffptr_setf(uint16_t rxoffptr)
{
    ASSERT_ERR((((uint32_t)rxoffptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL1_ADDR, (REG_BT_RD(BT_SPIPTRCNTL1_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)rxoffptr << 16));
}

__INLINE uint16_t bt_spiptrcntl1_rxonptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_spiptrcntl1_rxonptr_setf(uint16_t rxonptr)
{
    ASSERT_ERR((((uint32_t)rxonptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL1_ADDR, (REG_BT_RD(BT_SPIPTRCNTL1_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)rxonptr << 0));
}

/**
 * @brief SPIPTRCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          RXLENGTHPTR   0x0
 *  13:00              RSSIPTR   0x0
 * </pre>
 */
#define BT_SPIPTRCNTL2_ADDR   0x500904A8
#define BT_SPIPTRCNTL2_OFFSET 0x000000A8
#define BT_SPIPTRCNTL2_INDEX  0x0000002A
#define BT_SPIPTRCNTL2_RESET  0x00000000

__INLINE uint32_t bt_spiptrcntl2_get(void)
{
    return REG_BT_RD(BT_SPIPTRCNTL2_ADDR);
}

__INLINE void bt_spiptrcntl2_set(uint32_t value)
{
    REG_BT_WR(BT_SPIPTRCNTL2_ADDR, value);
}

// field definitions
#define BT_RXLENGTHPTR_MASK   ((uint32_t)0x3FFF0000)
#define BT_RXLENGTHPTR_LSB    16
#define BT_RXLENGTHPTR_WIDTH  ((uint32_t)0x0000000E)
#define BT_RSSIPTR_MASK       ((uint32_t)0x00003FFF)
#define BT_RSSIPTR_LSB        0
#define BT_RSSIPTR_WIDTH      ((uint32_t)0x0000000E)

#define BT_RXLENGTHPTR_RST    0x0
#define BT_RSSIPTR_RST        0x0

__INLINE void bt_spiptrcntl2_pack(uint16_t rxlengthptr, uint16_t rssiptr)
{
    ASSERT_ERR((((uint32_t)rxlengthptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)rssiptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL2_ADDR, ((uint32_t)rxlengthptr << 16) | ((uint32_t)rssiptr << 0));
}

__INLINE void bt_spiptrcntl2_unpack(uint16_t *rxlengthptr, uint16_t *rssiptr)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL2_ADDR);

    *rxlengthptr = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *rssiptr = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_spiptrcntl2_rxlengthptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_spiptrcntl2_rxlengthptr_setf(uint16_t rxlengthptr)
{
    ASSERT_ERR((((uint32_t)rxlengthptr << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL2_ADDR, (REG_BT_RD(BT_SPIPTRCNTL2_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)rxlengthptr << 16));
}

__INLINE uint16_t bt_spiptrcntl2_rssiptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_spiptrcntl2_rssiptr_setf(uint16_t rssiptr)
{
    ASSERT_ERR((((uint32_t)rssiptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL2_ADDR, (REG_BT_RD(BT_SPIPTRCNTL2_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)rssiptr << 0));
}

/**
 * @brief SPIPTRCNTL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00          RXPKTTYPPTR   0x0
 * </pre>
 */
#define BT_SPIPTRCNTL3_ADDR   0x500904AC
#define BT_SPIPTRCNTL3_OFFSET 0x000000AC
#define BT_SPIPTRCNTL3_INDEX  0x0000002B
#define BT_SPIPTRCNTL3_RESET  0x00000000

__INLINE uint32_t bt_spiptrcntl3_get(void)
{
    return REG_BT_RD(BT_SPIPTRCNTL3_ADDR);
}

__INLINE void bt_spiptrcntl3_set(uint32_t value)
{
    REG_BT_WR(BT_SPIPTRCNTL3_ADDR, value);
}

// field definitions
#define BT_RXPKTTYPPTR_MASK   ((uint32_t)0x00003FFF)
#define BT_RXPKTTYPPTR_LSB    0
#define BT_RXPKTTYPPTR_WIDTH  ((uint32_t)0x0000000E)

#define BT_RXPKTTYPPTR_RST    0x0

__INLINE uint16_t bt_spiptrcntl3_rxpkttypptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SPIPTRCNTL3_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_spiptrcntl3_rxpkttypptr_setf(uint16_t rxpkttypptr)
{
    ASSERT_ERR((((uint32_t)rxpkttypptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_SPIPTRCNTL3_ADDR, (uint32_t)rxpkttypptr << 0);
}

/**
 * @brief AESCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01             AES_MODE   0
 *     00            AES_START   0
 * </pre>
 */
#define BT_AESCNTL_ADDR   0x500904B0
#define BT_AESCNTL_OFFSET 0x000000B0
#define BT_AESCNTL_INDEX  0x0000002C
#define BT_AESCNTL_RESET  0x00000000

__INLINE uint32_t bt_aescntl_get(void)
{
    return REG_BT_RD(BT_AESCNTL_ADDR);
}

__INLINE void bt_aescntl_set(uint32_t value)
{
    REG_BT_WR(BT_AESCNTL_ADDR, value);
}

// field definitions
#define BT_AES_MODE_BIT     ((uint32_t)0x00000002)
#define BT_AES_MODE_POS     1
#define BT_AES_START_BIT    ((uint32_t)0x00000001)
#define BT_AES_START_POS    0

#define BT_AES_MODE_RST     0x0
#define BT_AES_START_RST    0x0

__INLINE void bt_aescntl_pack(uint8_t aesmode, uint8_t aesstart)
{
    ASSERT_ERR((((uint32_t)aesmode << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)aesstart << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AESCNTL_ADDR, ((uint32_t)aesmode << 1) | ((uint32_t)aesstart << 0));
}

__INLINE void bt_aescntl_unpack(uint8_t *aesmode, uint8_t *aesstart)
{
    uint32_t localVal = REG_BT_RD(BT_AESCNTL_ADDR);

    *aesmode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *aesstart = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_aescntl_aes_mode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AESCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_aescntl_aes_mode_setf(uint8_t aesmode)
{
    ASSERT_ERR((((uint32_t)aesmode << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_AESCNTL_ADDR, (REG_BT_RD(BT_AESCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)aesmode << 1));
}

__INLINE uint8_t bt_aescntl_aes_start_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AESCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_aescntl_aes_start_setf(uint8_t aesstart)
{
    ASSERT_ERR((((uint32_t)aesstart << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AESCNTL_ADDR, (REG_BT_RD(BT_AESCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)aesstart << 0));
}

/**
 * @brief AESKEY31_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           AESKEY31_0   0x0
 * </pre>
 */
#define BT_AESKEY31_0_ADDR   0x500904B4
#define BT_AESKEY31_0_OFFSET 0x000000B4
#define BT_AESKEY31_0_INDEX  0x0000002D
#define BT_AESKEY31_0_RESET  0x00000000

__INLINE uint32_t bt_aeskey31_0_get(void)
{
    return REG_BT_RD(BT_AESKEY31_0_ADDR);
}

__INLINE void bt_aeskey31_0_set(uint32_t value)
{
    REG_BT_WR(BT_AESKEY31_0_ADDR, value);
}

// field definitions
#define BT_AESKEY31_0_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_AESKEY31_0_LSB    0
#define BT_AESKEY31_0_WIDTH  ((uint32_t)0x00000020)

#define BT_AESKEY31_0_RST    0x0

__INLINE uint32_t bt_aeskey31_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AESKEY31_0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_aeskey31_0_setf(uint32_t aeskey310)
{
    ASSERT_ERR((((uint32_t)aeskey310 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BT_WR(BT_AESKEY31_0_ADDR, (uint32_t)aeskey310 << 0);
}

/**
 * @brief AESKEY63_32 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          AESKEY63_32   0x0
 * </pre>
 */
#define BT_AESKEY63_32_ADDR   0x500904B8
#define BT_AESKEY63_32_OFFSET 0x000000B8
#define BT_AESKEY63_32_INDEX  0x0000002E
#define BT_AESKEY63_32_RESET  0x00000000

__INLINE uint32_t bt_aeskey63_32_get(void)
{
    return REG_BT_RD(BT_AESKEY63_32_ADDR);
}

__INLINE void bt_aeskey63_32_set(uint32_t value)
{
    REG_BT_WR(BT_AESKEY63_32_ADDR, value);
}

// field definitions
#define BT_AESKEY63_32_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_AESKEY63_32_LSB    0
#define BT_AESKEY63_32_WIDTH  ((uint32_t)0x00000020)

#define BT_AESKEY63_32_RST    0x0

__INLINE uint32_t bt_aeskey63_32_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AESKEY63_32_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_aeskey63_32_setf(uint32_t aeskey6332)
{
    ASSERT_ERR((((uint32_t)aeskey6332 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BT_WR(BT_AESKEY63_32_ADDR, (uint32_t)aeskey6332 << 0);
}

/**
 * @brief AESKEY95_64 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          AESKEY95_64   0x0
 * </pre>
 */
#define BT_AESKEY95_64_ADDR   0x500904BC
#define BT_AESKEY95_64_OFFSET 0x000000BC
#define BT_AESKEY95_64_INDEX  0x0000002F
#define BT_AESKEY95_64_RESET  0x00000000

__INLINE uint32_t bt_aeskey95_64_get(void)
{
    return REG_BT_RD(BT_AESKEY95_64_ADDR);
}

__INLINE void bt_aeskey95_64_set(uint32_t value)
{
    REG_BT_WR(BT_AESKEY95_64_ADDR, value);
}

// field definitions
#define BT_AESKEY95_64_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_AESKEY95_64_LSB    0
#define BT_AESKEY95_64_WIDTH  ((uint32_t)0x00000020)

#define BT_AESKEY95_64_RST    0x0

__INLINE uint32_t bt_aeskey95_64_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AESKEY95_64_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_aeskey95_64_setf(uint32_t aeskey9564)
{
    ASSERT_ERR((((uint32_t)aeskey9564 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BT_WR(BT_AESKEY95_64_ADDR, (uint32_t)aeskey9564 << 0);
}

/**
 * @brief AESKEY127_96 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         AESKEY127_96   0x0
 * </pre>
 */
#define BT_AESKEY127_96_ADDR   0x500904C0
#define BT_AESKEY127_96_OFFSET 0x000000C0
#define BT_AESKEY127_96_INDEX  0x00000030
#define BT_AESKEY127_96_RESET  0x00000000

__INLINE uint32_t bt_aeskey127_96_get(void)
{
    return REG_BT_RD(BT_AESKEY127_96_ADDR);
}

__INLINE void bt_aeskey127_96_set(uint32_t value)
{
    REG_BT_WR(BT_AESKEY127_96_ADDR, value);
}

// field definitions
#define BT_AESKEY127_96_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_AESKEY127_96_LSB    0
#define BT_AESKEY127_96_WIDTH  ((uint32_t)0x00000020)

#define BT_AESKEY127_96_RST    0x0

__INLINE uint32_t bt_aeskey127_96_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AESKEY127_96_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_aeskey127_96_setf(uint32_t aeskey12796)
{
    ASSERT_ERR((((uint32_t)aeskey12796 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BT_WR(BT_AESKEY127_96_ADDR, (uint32_t)aeskey12796 << 0);
}

/**
 * @brief AESPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00               AESPTR   0x0
 * </pre>
 */
#define BT_AESPTR_ADDR   0x500904C4
#define BT_AESPTR_OFFSET 0x000000C4
#define BT_AESPTR_INDEX  0x00000031
#define BT_AESPTR_RESET  0x00000000

__INLINE uint32_t bt_aesptr_get(void)
{
    return REG_BT_RD(BT_AESPTR_ADDR);
}

__INLINE void bt_aesptr_set(uint32_t value)
{
    REG_BT_WR(BT_AESPTR_ADDR, value);
}

// field definitions
#define BT_AESPTR_MASK   ((uint32_t)0x00003FFF)
#define BT_AESPTR_LSB    0
#define BT_AESPTR_WIDTH  ((uint32_t)0x0000000E)

#define BT_AESPTR_RST    0x0

__INLINE uint16_t bt_aesptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AESPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_aesptr_setf(uint16_t aesptr)
{
    ASSERT_ERR((((uint32_t)aesptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_AESPTR_ADDR, (uint32_t)aesptr << 0);
}

/**
 * @brief TXMICVAL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             TXMICVAL   0x0
 * </pre>
 */
#define BT_TXMICVAL_ADDR   0x500904C8
#define BT_TXMICVAL_OFFSET 0x000000C8
#define BT_TXMICVAL_INDEX  0x00000032
#define BT_TXMICVAL_RESET  0x00000000

__INLINE uint32_t bt_txmicval_get(void)
{
    return REG_BT_RD(BT_TXMICVAL_ADDR);
}

// field definitions
#define BT_TXMICVAL_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_TXMICVAL_LSB    0
#define BT_TXMICVAL_WIDTH  ((uint32_t)0x00000020)

#define BT_TXMICVAL_RST    0x0

__INLINE uint32_t bt_txmicval_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_TXMICVAL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RXMICVAL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             RXMICVAL   0x0
 * </pre>
 */
#define BT_RXMICVAL_ADDR   0x500904CC
#define BT_RXMICVAL_OFFSET 0x000000CC
#define BT_RXMICVAL_INDEX  0x00000033
#define BT_RXMICVAL_RESET  0x00000000

__INLINE uint32_t bt_rxmicval_get(void)
{
    return REG_BT_RD(BT_RXMICVAL_ADDR);
}

// field definitions
#define BT_RXMICVAL_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_RXMICVAL_LSB    0
#define BT_RXMICVAL_WIDTH  ((uint32_t)0x00000020)

#define BT_RXMICVAL_RST    0x0

__INLINE uint32_t bt_rxmicval_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RXMICVAL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RFTESTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           INFINITERX   0
 *     27           RXPKTCNTEN   0
 *  26:24        PERCOUNT_MODE   0x0
 *     17             SSERRREN   0
 *     16              HERRREN   0
 *     15           INFINITETX   0
 *     13             PRBSTYPE   0
 *     12             TXPLDSRC   0
 *     11           TXPKTCNTEN   0
 * </pre>
 */
#define BT_RFTESTCNTL_ADDR   0x500904D0
#define BT_RFTESTCNTL_OFFSET 0x000000D0
#define BT_RFTESTCNTL_INDEX  0x00000034
#define BT_RFTESTCNTL_RESET  0x00000000

__INLINE uint32_t bt_rftestcntl_get(void)
{
    return REG_BT_RD(BT_RFTESTCNTL_ADDR);
}

__INLINE void bt_rftestcntl_set(uint32_t value)
{
    REG_BT_WR(BT_RFTESTCNTL_ADDR, value);
}

// field definitions
#define BT_INFINITERX_BIT       ((uint32_t)0x80000000)
#define BT_INFINITERX_POS       31
#define BT_RXPKTCNTEN_BIT       ((uint32_t)0x08000000)
#define BT_RXPKTCNTEN_POS       27
#define BT_PERCOUNT_MODE_MASK   ((uint32_t)0x07000000)
#define BT_PERCOUNT_MODE_LSB    24
#define BT_PERCOUNT_MODE_WIDTH  ((uint32_t)0x00000003)
#define BT_SSERRREN_BIT         ((uint32_t)0x00020000)
#define BT_SSERRREN_POS         17
#define BT_HERRREN_BIT          ((uint32_t)0x00010000)
#define BT_HERRREN_POS          16
#define BT_INFINITETX_BIT       ((uint32_t)0x00008000)
#define BT_INFINITETX_POS       15
#define BT_PRBSTYPE_BIT         ((uint32_t)0x00002000)
#define BT_PRBSTYPE_POS         13
#define BT_TXPLDSRC_BIT         ((uint32_t)0x00001000)
#define BT_TXPLDSRC_POS         12
#define BT_TXPKTCNTEN_BIT       ((uint32_t)0x00000800)
#define BT_TXPKTCNTEN_POS       11

#define BT_INFINITERX_RST       0x0
#define BT_RXPKTCNTEN_RST       0x0
#define BT_PERCOUNT_MODE_RST    0x0
#define BT_SSERRREN_RST         0x0
#define BT_HERRREN_RST          0x0
#define BT_INFINITETX_RST       0x0
#define BT_PRBSTYPE_RST         0x0
#define BT_TXPLDSRC_RST         0x0
#define BT_TXPKTCNTEN_RST       0x0

__INLINE void bt_rftestcntl_pack(uint8_t infiniterx, uint8_t rxpktcnten, uint8_t percountmode, uint8_t sserrren, uint8_t herrren, uint8_t infinitetx, uint8_t prbstype, uint8_t txpldsrc, uint8_t txpktcnten)
{
    ASSERT_ERR((((uint32_t)infiniterx << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)rxpktcnten << 27) & ~((uint32_t)0x08000000)) == 0);
    ASSERT_ERR((((uint32_t)percountmode << 24) & ~((uint32_t)0x07000000)) == 0);
    ASSERT_ERR((((uint32_t)sserrren << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)herrren << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)infinitetx << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)prbstype << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)txpldsrc << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)txpktcnten << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, ((uint32_t)infiniterx << 31) | ((uint32_t)rxpktcnten << 27) | ((uint32_t)percountmode << 24) | ((uint32_t)sserrren << 17) | ((uint32_t)herrren << 16) | ((uint32_t)infinitetx << 15) | ((uint32_t)prbstype << 13) | ((uint32_t)txpldsrc << 12) | ((uint32_t)txpktcnten << 11));
}

__INLINE void bt_rftestcntl_unpack(uint8_t *infiniterx, uint8_t *rxpktcnten, uint8_t *percountmode, uint8_t *sserrren, uint8_t *herrren, uint8_t *infinitetx, uint8_t *prbstype, uint8_t *txpldsrc, uint8_t *txpktcnten)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);

    *infiniterx = (localVal & ((uint32_t)0x80000000)) >> 31;
    *rxpktcnten = (localVal & ((uint32_t)0x08000000)) >> 27;
    *percountmode = (localVal & ((uint32_t)0x07000000)) >> 24;
    *sserrren = (localVal & ((uint32_t)0x00020000)) >> 17;
    *herrren = (localVal & ((uint32_t)0x00010000)) >> 16;
    *infinitetx = (localVal & ((uint32_t)0x00008000)) >> 15;
    *prbstype = (localVal & ((uint32_t)0x00002000)) >> 13;
    *txpldsrc = (localVal & ((uint32_t)0x00001000)) >> 12;
    *txpktcnten = (localVal & ((uint32_t)0x00000800)) >> 11;
}

__INLINE uint8_t bt_rftestcntl_infiniterx_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_rftestcntl_infiniterx_setf(uint8_t infiniterx)
{
    ASSERT_ERR((((uint32_t)infiniterx << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)infiniterx << 31));
}

__INLINE uint8_t bt_rftestcntl_rxpktcnten_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void bt_rftestcntl_rxpktcnten_setf(uint8_t rxpktcnten)
{
    ASSERT_ERR((((uint32_t)rxpktcnten << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)rxpktcnten << 27));
}

__INLINE uint8_t bt_rftestcntl_percount_mode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void bt_rftestcntl_percount_mode_setf(uint8_t percountmode)
{
    ASSERT_ERR((((uint32_t)percountmode << 24) & ~((uint32_t)0x07000000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)percountmode << 24));
}

__INLINE uint8_t bt_rftestcntl_sserrren_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void bt_rftestcntl_sserrren_setf(uint8_t sserrren)
{
    ASSERT_ERR((((uint32_t)sserrren << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)sserrren << 17));
}

__INLINE uint8_t bt_rftestcntl_herrren_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bt_rftestcntl_herrren_setf(uint8_t herrren)
{
    ASSERT_ERR((((uint32_t)herrren << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)herrren << 16));
}

__INLINE uint8_t bt_rftestcntl_infinitetx_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_rftestcntl_infinitetx_setf(uint8_t infinitetx)
{
    ASSERT_ERR((((uint32_t)infinitetx << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)infinitetx << 15));
}

__INLINE uint8_t bt_rftestcntl_prbstype_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_rftestcntl_prbstype_setf(uint8_t prbstype)
{
    ASSERT_ERR((((uint32_t)prbstype << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)prbstype << 13));
}

__INLINE uint8_t bt_rftestcntl_txpldsrc_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_rftestcntl_txpldsrc_setf(uint8_t txpldsrc)
{
    ASSERT_ERR((((uint32_t)txpldsrc << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)txpldsrc << 12));
}

__INLINE uint8_t bt_rftestcntl_txpktcnten_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void bt_rftestcntl_txpktcnten_setf(uint8_t txpktcnten)
{
    ASSERT_ERR((((uint32_t)txpktcnten << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BT_WR(BT_RFTESTCNTL_ADDR, (REG_BT_RD(BT_RFTESTCNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)txpktcnten << 11));
}

/**
 * @brief RFTESTFREQ register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     18         LOOPBACKMODE   0
 *     17     DIRECTLOOPBACKEN   0
 *     16           TESTMODEEN   0
 *  14:08               RXFREQ   0x0
 *  06:00               TXFREQ   0x0
 * </pre>
 */
#define BT_RFTESTFREQ_ADDR   0x500904D4
#define BT_RFTESTFREQ_OFFSET 0x000000D4
#define BT_RFTESTFREQ_INDEX  0x00000035
#define BT_RFTESTFREQ_RESET  0x00000000

__INLINE uint32_t bt_rftestfreq_get(void)
{
    return REG_BT_RD(BT_RFTESTFREQ_ADDR);
}

__INLINE void bt_rftestfreq_set(uint32_t value)
{
    REG_BT_WR(BT_RFTESTFREQ_ADDR, value);
}

// field definitions
#define BT_LOOPBACKMODE_BIT        ((uint32_t)0x00040000)
#define BT_LOOPBACKMODE_POS        18
#define BT_DIRECTLOOPBACKEN_BIT    ((uint32_t)0x00020000)
#define BT_DIRECTLOOPBACKEN_POS    17
#define BT_TESTMODEEN_BIT          ((uint32_t)0x00010000)
#define BT_TESTMODEEN_POS          16
#define BT_RXFREQ_MASK             ((uint32_t)0x00007F00)
#define BT_RXFREQ_LSB              8
#define BT_RXFREQ_WIDTH            ((uint32_t)0x00000007)
#define BT_TXFREQ_MASK             ((uint32_t)0x0000007F)
#define BT_TXFREQ_LSB              0
#define BT_TXFREQ_WIDTH            ((uint32_t)0x00000007)

#define BT_LOOPBACKMODE_RST        0x0
#define BT_DIRECTLOOPBACKEN_RST    0x0
#define BT_TESTMODEEN_RST          0x0
#define BT_RXFREQ_RST              0x0
#define BT_TXFREQ_RST              0x0

__INLINE void bt_rftestfreq_pack(uint8_t loopbackmode, uint8_t directloopbacken, uint8_t testmodeen, uint8_t rxfreq, uint8_t txfreq)
{
    ASSERT_ERR((((uint32_t)loopbackmode << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)directloopbacken << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)testmodeen << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)rxfreq << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txfreq << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_RFTESTFREQ_ADDR, ((uint32_t)loopbackmode << 18) | ((uint32_t)directloopbacken << 17) | ((uint32_t)testmodeen << 16) | ((uint32_t)rxfreq << 8) | ((uint32_t)txfreq << 0));
}

__INLINE void bt_rftestfreq_unpack(uint8_t *loopbackmode, uint8_t *directloopbacken, uint8_t *testmodeen, uint8_t *rxfreq, uint8_t *txfreq)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTFREQ_ADDR);

    *loopbackmode = (localVal & ((uint32_t)0x00040000)) >> 18;
    *directloopbacken = (localVal & ((uint32_t)0x00020000)) >> 17;
    *testmodeen = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rxfreq = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txfreq = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t bt_rftestfreq_loopbackmode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTFREQ_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void bt_rftestfreq_loopbackmode_setf(uint8_t loopbackmode)
{
    ASSERT_ERR((((uint32_t)loopbackmode << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_BT_WR(BT_RFTESTFREQ_ADDR, (REG_BT_RD(BT_RFTESTFREQ_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)loopbackmode << 18));
}

__INLINE uint8_t bt_rftestfreq_directloopbacken_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTFREQ_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void bt_rftestfreq_directloopbacken_setf(uint8_t directloopbacken)
{
    ASSERT_ERR((((uint32_t)directloopbacken << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_BT_WR(BT_RFTESTFREQ_ADDR, (REG_BT_RD(BT_RFTESTFREQ_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)directloopbacken << 17));
}

__INLINE uint8_t bt_rftestfreq_testmodeen_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTFREQ_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bt_rftestfreq_testmodeen_setf(uint8_t testmodeen)
{
    ASSERT_ERR((((uint32_t)testmodeen << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BT_WR(BT_RFTESTFREQ_ADDR, (REG_BT_RD(BT_RFTESTFREQ_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)testmodeen << 16));
}

__INLINE uint8_t bt_rftestfreq_rxfreq_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTFREQ_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void bt_rftestfreq_rxfreq_setf(uint8_t rxfreq)
{
    ASSERT_ERR((((uint32_t)rxfreq << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BT_WR(BT_RFTESTFREQ_ADDR, (REG_BT_RD(BT_RFTESTFREQ_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)rxfreq << 8));
}

__INLINE uint8_t bt_rftestfreq_txfreq_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTFREQ_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void bt_rftestfreq_txfreq_setf(uint8_t txfreq)
{
    ASSERT_ERR((((uint32_t)txfreq << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_RFTESTFREQ_ADDR, (REG_BT_RD(BT_RFTESTFREQ_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)txfreq << 0));
}

/**
 * @brief RFTESTTXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             TXPKTCNT   0x0
 * </pre>
 */
#define BT_RFTESTTXSTAT_ADDR   0x500904D8
#define BT_RFTESTTXSTAT_OFFSET 0x000000D8
#define BT_RFTESTTXSTAT_INDEX  0x00000036
#define BT_RFTESTTXSTAT_RESET  0x00000000

__INLINE uint32_t bt_rftesttxstat_get(void)
{
    return REG_BT_RD(BT_RFTESTTXSTAT_ADDR);
}

// field definitions
#define BT_TXPKTCNT_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_TXPKTCNT_LSB    0
#define BT_TXPKTCNT_WIDTH  ((uint32_t)0x00000020)

#define BT_TXPKTCNT_RST    0x0

__INLINE uint32_t bt_rftesttxstat_txpktcnt_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTTXSTAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RFTESTRXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             RXPKTCNT   0x0
 * </pre>
 */
#define BT_RFTESTRXSTAT_ADDR   0x500904DC
#define BT_RFTESTRXSTAT_OFFSET 0x000000DC
#define BT_RFTESTRXSTAT_INDEX  0x00000037
#define BT_RFTESTRXSTAT_RESET  0x00000000

__INLINE uint32_t bt_rftestrxstat_get(void)
{
    return REG_BT_RD(BT_RFTESTRXSTAT_ADDR);
}

// field definitions
#define BT_RXPKTCNT_MASK   ((uint32_t)0xFFFFFFFF)
#define BT_RXPKTCNT_LSB    0
#define BT_RXPKTCNT_WIDTH  ((uint32_t)0x00000020)

#define BT_RXPKTCNT_RST    0x0

__INLINE uint32_t bt_rftestrxstat_rxpktcnt_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_RFTESTRXSTAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief TIMGENCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:16   PREFETCHABORT_TIME   0x1DF
 *  08:00        PREFETCH_TIME   0xBD
 * </pre>
 */
#define BT_TIMGENCNTL_ADDR   0x500904E0
#define BT_TIMGENCNTL_OFFSET 0x000000E0
#define BT_TIMGENCNTL_INDEX  0x00000038
#define BT_TIMGENCNTL_RESET  0x01DF00BD

__INLINE uint32_t bt_timgencntl_get(void)
{
    return REG_BT_RD(BT_TIMGENCNTL_ADDR);
}

__INLINE void bt_timgencntl_set(uint32_t value)
{
    REG_BT_WR(BT_TIMGENCNTL_ADDR, value);
}

// field definitions
#define BT_PREFETCHABORT_TIME_MASK   ((uint32_t)0x03FF0000)
#define BT_PREFETCHABORT_TIME_LSB    16
#define BT_PREFETCHABORT_TIME_WIDTH  ((uint32_t)0x0000000A)
#define BT_PREFETCH_TIME_MASK        ((uint32_t)0x000001FF)
#define BT_PREFETCH_TIME_LSB         0
#define BT_PREFETCH_TIME_WIDTH       ((uint32_t)0x00000009)

#define BT_PREFETCHABORT_TIME_RST    0x1DF
#define BT_PREFETCH_TIME_RST         0xBD

__INLINE void bt_timgencntl_pack(uint16_t prefetchaborttime, uint16_t prefetchtime)
{
    ASSERT_ERR((((uint32_t)prefetchaborttime << 16) & ~((uint32_t)0x03FF0000)) == 0);
    ASSERT_ERR((((uint32_t)prefetchtime << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BT_WR(BT_TIMGENCNTL_ADDR, ((uint32_t)prefetchaborttime << 16) | ((uint32_t)prefetchtime << 0));
}

__INLINE void bt_timgencntl_unpack(uint16_t *prefetchaborttime, uint16_t *prefetchtime)
{
    uint32_t localVal = REG_BT_RD(BT_TIMGENCNTL_ADDR);

    *prefetchaborttime = (localVal & ((uint32_t)0x03FF0000)) >> 16;
    *prefetchtime = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t bt_timgencntl_prefetchabort_time_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_TIMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x03FF0000)) >> 16);
}

__INLINE void bt_timgencntl_prefetchabort_time_setf(uint16_t prefetchaborttime)
{
    ASSERT_ERR((((uint32_t)prefetchaborttime << 16) & ~((uint32_t)0x03FF0000)) == 0);
    REG_BT_WR(BT_TIMGENCNTL_ADDR, (REG_BT_RD(BT_TIMGENCNTL_ADDR) & ~((uint32_t)0x03FF0000)) | ((uint32_t)prefetchaborttime << 16));
}

__INLINE uint16_t bt_timgencntl_prefetch_time_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_TIMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void bt_timgencntl_prefetch_time_setf(uint16_t prefetchtime)
{
    ASSERT_ERR((((uint32_t)prefetchtime << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BT_WR(BT_TIMGENCNTL_ADDR, (REG_BT_RD(BT_TIMGENCNTL_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)prefetchtime << 0));
}

/**
 * @brief FINETIMTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00           FINETARGET   0x0
 * </pre>
 */
#define BT_FINETIMTGT_ADDR   0x500904E4
#define BT_FINETIMTGT_OFFSET 0x000000E4
#define BT_FINETIMTGT_INDEX  0x00000039
#define BT_FINETIMTGT_RESET  0x00000000

__INLINE uint32_t bt_finetimtgt_get(void)
{
    return REG_BT_RD(BT_FINETIMTGT_ADDR);
}

__INLINE void bt_finetimtgt_set(uint32_t value)
{
    REG_BT_WR(BT_FINETIMTGT_ADDR, value);
}

// field definitions
#define BT_FINETARGET_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_FINETARGET_LSB    0
#define BT_FINETARGET_WIDTH  ((uint32_t)0x0000001C)

#define BT_FINETARGET_RST    0x0

__INLINE uint32_t bt_finetimtgt_finetarget_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_FINETIMTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_finetimtgt_finetarget_setf(uint32_t finetarget)
{
    ASSERT_ERR((((uint32_t)finetarget << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_FINETIMTGT_ADDR, (uint32_t)finetarget << 0);
}

/**
 * @brief CLKNTGT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00             CLKNTGT1   0x0
 * </pre>
 */
#define BT_CLKNTGT1_ADDR   0x500904E8
#define BT_CLKNTGT1_OFFSET 0x000000E8
#define BT_CLKNTGT1_INDEX  0x0000003A
#define BT_CLKNTGT1_RESET  0x00000000

__INLINE uint32_t bt_clkntgt1_get(void)
{
    return REG_BT_RD(BT_CLKNTGT1_ADDR);
}

__INLINE void bt_clkntgt1_set(uint32_t value)
{
    REG_BT_WR(BT_CLKNTGT1_ADDR, value);
}

// field definitions
#define BT_CLKNTGT1_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_CLKNTGT1_LSB    0
#define BT_CLKNTGT1_WIDTH  ((uint32_t)0x0000001C)

#define BT_CLKNTGT1_RST    0x0

__INLINE uint32_t bt_clkntgt1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_CLKNTGT1_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_clkntgt1_setf(uint32_t clkntgt1)
{
    ASSERT_ERR((((uint32_t)clkntgt1 << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_CLKNTGT1_ADDR, (uint32_t)clkntgt1 << 0);
}

/**
 * @brief HMICROSECTGT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00        HMICROSECTGT1   0x0
 * </pre>
 */
#define BT_HMICROSECTGT1_ADDR   0x500904EC
#define BT_HMICROSECTGT1_OFFSET 0x000000EC
#define BT_HMICROSECTGT1_INDEX  0x0000003B
#define BT_HMICROSECTGT1_RESET  0x00000000

__INLINE uint32_t bt_hmicrosectgt1_get(void)
{
    return REG_BT_RD(BT_HMICROSECTGT1_ADDR);
}

__INLINE void bt_hmicrosectgt1_set(uint32_t value)
{
    REG_BT_WR(BT_HMICROSECTGT1_ADDR, value);
}

// field definitions
#define BT_HMICROSECTGT1_MASK   ((uint32_t)0x000003FF)
#define BT_HMICROSECTGT1_LSB    0
#define BT_HMICROSECTGT1_WIDTH  ((uint32_t)0x0000000A)

#define BT_HMICROSECTGT1_RST    0x0

__INLINE uint16_t bt_hmicrosectgt1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_HMICROSECTGT1_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_hmicrosectgt1_setf(uint16_t hmicrosectgt1)
{
    ASSERT_ERR((((uint32_t)hmicrosectgt1 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BT_WR(BT_HMICROSECTGT1_ADDR, (uint32_t)hmicrosectgt1 << 0);
}

/**
 * @brief CLKNTGT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00             CLKNTGT2   0x0
 * </pre>
 */
#define BT_CLKNTGT2_ADDR   0x500904F0
#define BT_CLKNTGT2_OFFSET 0x000000F0
#define BT_CLKNTGT2_INDEX  0x0000003C
#define BT_CLKNTGT2_RESET  0x00000000

__INLINE uint32_t bt_clkntgt2_get(void)
{
    return REG_BT_RD(BT_CLKNTGT2_ADDR);
}

__INLINE void bt_clkntgt2_set(uint32_t value)
{
    REG_BT_WR(BT_CLKNTGT2_ADDR, value);
}

// field definitions
#define BT_CLKNTGT2_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_CLKNTGT2_LSB    0
#define BT_CLKNTGT2_WIDTH  ((uint32_t)0x0000001C)

#define BT_CLKNTGT2_RST    0x0

__INLINE uint32_t bt_clkntgt2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_CLKNTGT2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_clkntgt2_setf(uint32_t clkntgt2)
{
    ASSERT_ERR((((uint32_t)clkntgt2 << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_CLKNTGT2_ADDR, (uint32_t)clkntgt2 << 0);
}

/**
 * @brief HMICROSECTGT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00        HMICROSECTGT2   0x0
 * </pre>
 */
#define BT_HMICROSECTGT2_ADDR   0x500904F4
#define BT_HMICROSECTGT2_OFFSET 0x000000F4
#define BT_HMICROSECTGT2_INDEX  0x0000003D
#define BT_HMICROSECTGT2_RESET  0x00000000

__INLINE uint32_t bt_hmicrosectgt2_get(void)
{
    return REG_BT_RD(BT_HMICROSECTGT2_ADDR);
}

__INLINE void bt_hmicrosectgt2_set(uint32_t value)
{
    REG_BT_WR(BT_HMICROSECTGT2_ADDR, value);
}

// field definitions
#define BT_HMICROSECTGT2_MASK   ((uint32_t)0x000003FF)
#define BT_HMICROSECTGT2_LSB    0
#define BT_HMICROSECTGT2_WIDTH  ((uint32_t)0x0000000A)

#define BT_HMICROSECTGT2_RST    0x0

__INLINE uint16_t bt_hmicrosectgt2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_HMICROSECTGT2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_hmicrosectgt2_setf(uint16_t hmicrosectgt2)
{
    ASSERT_ERR((((uint32_t)hmicrosectgt2 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BT_WR(BT_HMICROSECTGT2_ADDR, (uint32_t)hmicrosectgt2 << 0);
}

/**
 * @brief CLKNTGT3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00             CLKNTGT3   0x0
 * </pre>
 */
#define BT_CLKNTGT3_ADDR   0x500904F8
#define BT_CLKNTGT3_OFFSET 0x000000F8
#define BT_CLKNTGT3_INDEX  0x0000003E
#define BT_CLKNTGT3_RESET  0x00000000

__INLINE uint32_t bt_clkntgt3_get(void)
{
    return REG_BT_RD(BT_CLKNTGT3_ADDR);
}

__INLINE void bt_clkntgt3_set(uint32_t value)
{
    REG_BT_WR(BT_CLKNTGT3_ADDR, value);
}

// field definitions
#define BT_CLKNTGT3_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_CLKNTGT3_LSB    0
#define BT_CLKNTGT3_WIDTH  ((uint32_t)0x0000001C)

#define BT_CLKNTGT3_RST    0x0

__INLINE uint32_t bt_clkntgt3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_CLKNTGT3_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_clkntgt3_setf(uint32_t clkntgt3)
{
    ASSERT_ERR((((uint32_t)clkntgt3 << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_CLKNTGT3_ADDR, (uint32_t)clkntgt3 << 0);
}

/**
 * @brief HMICROSECTGT3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00        HMICROSECTGT3   0x0
 * </pre>
 */
#define BT_HMICROSECTGT3_ADDR   0x500904FC
#define BT_HMICROSECTGT3_OFFSET 0x000000FC
#define BT_HMICROSECTGT3_INDEX  0x0000003F
#define BT_HMICROSECTGT3_RESET  0x00000000

__INLINE uint32_t bt_hmicrosectgt3_get(void)
{
    return REG_BT_RD(BT_HMICROSECTGT3_ADDR);
}

__INLINE void bt_hmicrosectgt3_set(uint32_t value)
{
    REG_BT_WR(BT_HMICROSECTGT3_ADDR, value);
}

// field definitions
#define BT_HMICROSECTGT3_MASK   ((uint32_t)0x000003FF)
#define BT_HMICROSECTGT3_LSB    0
#define BT_HMICROSECTGT3_WIDTH  ((uint32_t)0x0000000A)

#define BT_HMICROSECTGT3_RST    0x0

__INLINE uint16_t bt_hmicrosectgt3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_HMICROSECTGT3_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_hmicrosectgt3_setf(uint16_t hmicrosectgt3)
{
    ASSERT_ERR((((uint32_t)hmicrosectgt3 << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BT_WR(BT_HMICROSECTGT3_ADDR, (uint32_t)hmicrosectgt3 << 0);
}

/**
 * @brief SLOTCLK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                 SAMP   0
 *     30             CLKN_UPD   0
 *  27:00                 SCLK   0x0
 * </pre>
 */
#define BT_SLOTCLK_ADDR   0x50090500
#define BT_SLOTCLK_OFFSET 0x00000100
#define BT_SLOTCLK_INDEX  0x00000040
#define BT_SLOTCLK_RESET  0x00000000

__INLINE uint32_t bt_slotclk_get(void)
{
    return REG_BT_RD(BT_SLOTCLK_ADDR);
}

__INLINE void bt_slotclk_set(uint32_t value)
{
    REG_BT_WR(BT_SLOTCLK_ADDR, value);
}

// field definitions
#define BT_SAMP_BIT        ((uint32_t)0x80000000)
#define BT_SAMP_POS        31
#define BT_CLKN_UPD_BIT    ((uint32_t)0x40000000)
#define BT_CLKN_UPD_POS    30
#define BT_SCLK_MASK       ((uint32_t)0x0FFFFFFF)
#define BT_SCLK_LSB        0
#define BT_SCLK_WIDTH      ((uint32_t)0x0000001C)

#define BT_SAMP_RST        0x0
#define BT_CLKN_UPD_RST    0x0
#define BT_SCLK_RST        0x0

__INLINE void bt_slotclk_pack(uint8_t samp, uint8_t clknupd, uint32_t sclk)
{
    ASSERT_ERR((((uint32_t)samp << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)clknupd << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)sclk << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_SLOTCLK_ADDR, ((uint32_t)samp << 31) | ((uint32_t)clknupd << 30) | ((uint32_t)sclk << 0));
}

__INLINE void bt_slotclk_unpack(uint8_t *samp, uint8_t *clknupd, uint32_t *sclk)
{
    uint32_t localVal = REG_BT_RD(BT_SLOTCLK_ADDR);

    *samp = (localVal & ((uint32_t)0x80000000)) >> 31;
    *clknupd = (localVal & ((uint32_t)0x40000000)) >> 30;
    *sclk = (localVal & ((uint32_t)0x0FFFFFFF)) >> 0;
}

__INLINE uint8_t bt_slotclk_samp_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_slotclk_samp_setf(uint8_t samp)
{
    ASSERT_ERR((((uint32_t)samp << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_SLOTCLK_ADDR, (REG_BT_RD(BT_SLOTCLK_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)samp << 31));
}

__INLINE uint8_t bt_slotclk_clkn_upd_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void bt_slotclk_clkn_upd_setf(uint8_t clknupd)
{
    ASSERT_ERR((((uint32_t)clknupd << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BT_WR(BT_SLOTCLK_ADDR, (REG_BT_RD(BT_SLOTCLK_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)clknupd << 30));
}

__INLINE uint32_t bt_slotclk_sclk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x0FFFFFFF)) >> 0);
}

__INLINE void bt_slotclk_sclk_setf(uint32_t sclk)
{
    ASSERT_ERR((((uint32_t)sclk << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BT_WR(BT_SLOTCLK_ADDR, (REG_BT_RD(BT_SLOTCLK_ADDR) & ~((uint32_t)0x0FFFFFFF)) | ((uint32_t)sclk << 0));
}

/**
 * @brief FINETIMECNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00              FINECNT   0x0
 * </pre>
 */
#define BT_FINETIMECNT_ADDR   0x50090504
#define BT_FINETIMECNT_OFFSET 0x00000104
#define BT_FINETIMECNT_INDEX  0x00000041
#define BT_FINETIMECNT_RESET  0x00000000

__INLINE uint32_t bt_finetimecnt_get(void)
{
    return REG_BT_RD(BT_FINETIMECNT_ADDR);
}

// field definitions
#define BT_FINECNT_MASK   ((uint32_t)0x000003FF)
#define BT_FINECNT_LSB    0
#define BT_FINECNT_WIDTH  ((uint32_t)0x0000000A)

#define BT_FINECNT_RST    0x0

__INLINE uint16_t bt_finetimecnt_finecnt_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_FINETIMECNT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ACTSCHCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            START_ACT   0
 *  03:00            ENTRY_IDX   0x0
 * </pre>
 */
#define BT_ACTSCHCNTL_ADDR   0x50090510
#define BT_ACTSCHCNTL_OFFSET 0x00000110
#define BT_ACTSCHCNTL_INDEX  0x00000044
#define BT_ACTSCHCNTL_RESET  0x00000000

__INLINE uint32_t bt_actschcntl_get(void)
{
    return REG_BT_RD(BT_ACTSCHCNTL_ADDR);
}

__INLINE void bt_actschcntl_set(uint32_t value)
{
    REG_BT_WR(BT_ACTSCHCNTL_ADDR, value);
}

// field definitions
#define BT_START_ACT_BIT    ((uint32_t)0x80000000)
#define BT_START_ACT_POS    31
#define BT_ENTRY_IDX_MASK   ((uint32_t)0x0000000F)
#define BT_ENTRY_IDX_LSB    0
#define BT_ENTRY_IDX_WIDTH  ((uint32_t)0x00000004)

#define BT_START_ACT_RST    0x0
#define BT_ENTRY_IDX_RST    0x0

__INLINE void bt_actschcntl_pack(uint8_t startact, uint8_t entryidx)
{
    ASSERT_ERR((((uint32_t)startact << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)entryidx << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_ACTSCHCNTL_ADDR, ((uint32_t)startact << 31) | ((uint32_t)entryidx << 0));
}

__INLINE void bt_actschcntl_unpack(uint8_t *startact, uint8_t *entryidx)
{
    uint32_t localVal = REG_BT_RD(BT_ACTSCHCNTL_ADDR);

    *startact = (localVal & ((uint32_t)0x80000000)) >> 31;
    *entryidx = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_actschcntl_start_act_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTSCHCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_actschcntl_start_act_setf(uint8_t startact)
{
    ASSERT_ERR((((uint32_t)startact << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_ACTSCHCNTL_ADDR, (REG_BT_RD(BT_ACTSCHCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)startact << 31));
}

__INLINE uint8_t bt_actschcntl_entry_idx_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ACTSCHCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_actschcntl_entry_idx_setf(uint8_t entryidx)
{
    ASSERT_ERR((((uint32_t)entryidx << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_ACTSCHCNTL_ADDR, (REG_BT_RD(BT_ACTSCHCNTL_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)entryidx << 0));
}

/**
 * @brief STARTFRMCLKNTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00       STARTFRMCLKNTS   0x0
 * </pre>
 */
#define BT_STARTFRMCLKNTS_ADDR   0x50090514
#define BT_STARTFRMCLKNTS_OFFSET 0x00000114
#define BT_STARTFRMCLKNTS_INDEX  0x00000045
#define BT_STARTFRMCLKNTS_RESET  0x00000000

__INLINE uint32_t bt_startfrmclknts_get(void)
{
    return REG_BT_RD(BT_STARTFRMCLKNTS_ADDR);
}

// field definitions
#define BT_STARTFRMCLKNTS_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_STARTFRMCLKNTS_LSB    0
#define BT_STARTFRMCLKNTS_WIDTH  ((uint32_t)0x0000001C)

#define BT_STARTFRMCLKNTS_RST    0x0

__INLINE uint32_t bt_startfrmclknts_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_STARTFRMCLKNTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief STARTFRMFINECNTTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00    STARTFRMFINECNTTS   0x0
 * </pre>
 */
#define BT_STARTFRMFINECNTTS_ADDR   0x50090518
#define BT_STARTFRMFINECNTTS_OFFSET 0x00000118
#define BT_STARTFRMFINECNTTS_INDEX  0x00000046
#define BT_STARTFRMFINECNTTS_RESET  0x00000000

__INLINE uint32_t bt_startfrmfinecntts_get(void)
{
    return REG_BT_RD(BT_STARTFRMFINECNTTS_ADDR);
}

// field definitions
#define BT_STARTFRMFINECNTTS_MASK   ((uint32_t)0x000003FF)
#define BT_STARTFRMFINECNTTS_LSB    0
#define BT_STARTFRMFINECNTTS_WIDTH  ((uint32_t)0x0000000A)

#define BT_STARTFRMFINECNTTS_RST    0x0

__INLINE uint16_t bt_startfrmfinecntts_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_STARTFRMFINECNTTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ENDFRMCLKNTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00         ENDFRMCLKNTS   0x0
 * </pre>
 */
#define BT_ENDFRMCLKNTS_ADDR   0x5009051C
#define BT_ENDFRMCLKNTS_OFFSET 0x0000011C
#define BT_ENDFRMCLKNTS_INDEX  0x00000047
#define BT_ENDFRMCLKNTS_RESET  0x00000000

__INLINE uint32_t bt_endfrmclknts_get(void)
{
    return REG_BT_RD(BT_ENDFRMCLKNTS_ADDR);
}

// field definitions
#define BT_ENDFRMCLKNTS_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_ENDFRMCLKNTS_LSB    0
#define BT_ENDFRMCLKNTS_WIDTH  ((uint32_t)0x0000001C)

#define BT_ENDFRMCLKNTS_RST    0x0

__INLINE uint32_t bt_endfrmclknts_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ENDFRMCLKNTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ENDFRMFINECNTTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00      ENDFRMFINECNTTS   0x0
 * </pre>
 */
#define BT_ENDFRMFINECNTTS_ADDR   0x50090520
#define BT_ENDFRMFINECNTTS_OFFSET 0x00000120
#define BT_ENDFRMFINECNTTS_INDEX  0x00000048
#define BT_ENDFRMFINECNTTS_RESET  0x00000000

__INLINE uint32_t bt_endfrmfinecntts_get(void)
{
    return REG_BT_RD(BT_ENDFRMFINECNTTS_ADDR);
}

// field definitions
#define BT_ENDFRMFINECNTTS_MASK   ((uint32_t)0x000003FF)
#define BT_ENDFRMFINECNTTS_LSB    0
#define BT_ENDFRMFINECNTTS_WIDTH  ((uint32_t)0x0000000A)

#define BT_ENDFRMFINECNTTS_RST    0x0

__INLINE uint16_t bt_endfrmfinecntts_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ENDFRMFINECNTTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief SKIPFRMCLKNTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00       SKIPFRMPCLKNTS   0x0
 * </pre>
 */
#define BT_SKIPFRMCLKNTS_ADDR   0x50090524
#define BT_SKIPFRMCLKNTS_OFFSET 0x00000124
#define BT_SKIPFRMCLKNTS_INDEX  0x00000049
#define BT_SKIPFRMCLKNTS_RESET  0x00000000

__INLINE uint32_t bt_skipfrmclknts_get(void)
{
    return REG_BT_RD(BT_SKIPFRMCLKNTS_ADDR);
}

// field definitions
#define BT_SKIPFRMPCLKNTS_MASK   ((uint32_t)0x0FFFFFFF)
#define BT_SKIPFRMPCLKNTS_LSB    0
#define BT_SKIPFRMPCLKNTS_WIDTH  ((uint32_t)0x0000001C)

#define BT_SKIPFRMPCLKNTS_RST    0x0

__INLINE uint32_t bt_skipfrmclknts_skipfrmpclknts_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SKIPFRMCLKNTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief SKIPFRMFINECNTTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00     SKIPFRMFINECNTTS   0x0
 * </pre>
 */
#define BT_SKIPFRMFINECNTTS_ADDR   0x50090528
#define BT_SKIPFRMFINECNTTS_OFFSET 0x00000128
#define BT_SKIPFRMFINECNTTS_INDEX  0x0000004A
#define BT_SKIPFRMFINECNTTS_RESET  0x00000000

__INLINE uint32_t bt_skipfrmfinecntts_get(void)
{
    return REG_BT_RD(BT_SKIPFRMFINECNTTS_ADDR);
}

// field definitions
#define BT_SKIPFRMFINECNTTS_MASK   ((uint32_t)0x000003FF)
#define BT_SKIPFRMFINECNTTS_LSB    0
#define BT_SKIPFRMFINECNTTS_WIDTH  ((uint32_t)0x0000000A)

#define BT_SKIPFRMFINECNTTS_RST    0x0

__INLINE uint16_t bt_skipfrmfinecntts_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_SKIPFRMFINECNTTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ABTRAINCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            ABTPAGEEN   0
 *     30    ABTPAGESTARTVALUE   0
 *     28          ABTPAGELOAD   0
 *  26:16          ABTPAGETIME   0x100
 *     15             ABTINQEN   0
 *     14     ABTINQSTARTVALUE   0
 *     12           ABTINQLOAD   0
 *  10:00           ABTINQTIME   0x100
 * </pre>
 */
#define BT_ABTRAINCNTL_ADDR   0x50090530
#define BT_ABTRAINCNTL_OFFSET 0x00000130
#define BT_ABTRAINCNTL_INDEX  0x0000004C
#define BT_ABTRAINCNTL_RESET  0x01000100

__INLINE uint32_t bt_abtraincntl_get(void)
{
    return REG_BT_RD(BT_ABTRAINCNTL_ADDR);
}

__INLINE void bt_abtraincntl_set(uint32_t value)
{
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, value);
}

// field definitions
#define BT_ABTPAGEEN_BIT            ((uint32_t)0x80000000)
#define BT_ABTPAGEEN_POS            31
#define BT_ABTPAGESTARTVALUE_BIT    ((uint32_t)0x40000000)
#define BT_ABTPAGESTARTVALUE_POS    30
#define BT_ABTPAGELOAD_BIT          ((uint32_t)0x10000000)
#define BT_ABTPAGELOAD_POS          28
#define BT_ABTPAGETIME_MASK         ((uint32_t)0x07FF0000)
#define BT_ABTPAGETIME_LSB          16
#define BT_ABTPAGETIME_WIDTH        ((uint32_t)0x0000000B)
#define BT_ABTINQEN_BIT             ((uint32_t)0x00008000)
#define BT_ABTINQEN_POS             15
#define BT_ABTINQSTARTVALUE_BIT     ((uint32_t)0x00004000)
#define BT_ABTINQSTARTVALUE_POS     14
#define BT_ABTINQLOAD_BIT           ((uint32_t)0x00001000)
#define BT_ABTINQLOAD_POS           12
#define BT_ABTINQTIME_MASK          ((uint32_t)0x000007FF)
#define BT_ABTINQTIME_LSB           0
#define BT_ABTINQTIME_WIDTH         ((uint32_t)0x0000000B)

#define BT_ABTPAGEEN_RST            0x0
#define BT_ABTPAGESTARTVALUE_RST    0x0
#define BT_ABTPAGELOAD_RST          0x0
#define BT_ABTPAGETIME_RST          0x100
#define BT_ABTINQEN_RST             0x0
#define BT_ABTINQSTARTVALUE_RST     0x0
#define BT_ABTINQLOAD_RST           0x0
#define BT_ABTINQTIME_RST           0x100

__INLINE void bt_abtraincntl_pack(uint8_t abtpageen, uint8_t abtpagestartvalue, uint8_t abtpageload, uint16_t abtpagetime, uint8_t abtinqen, uint8_t abtinqstartvalue, uint8_t abtinqload, uint16_t abtinqtime)
{
    ASSERT_ERR((((uint32_t)abtpageen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)abtpagestartvalue << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)abtpageload << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)abtpagetime << 16) & ~((uint32_t)0x07FF0000)) == 0);
    ASSERT_ERR((((uint32_t)abtinqen << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)abtinqstartvalue << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)abtinqload << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)abtinqtime << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, ((uint32_t)abtpageen << 31) | ((uint32_t)abtpagestartvalue << 30) | ((uint32_t)abtpageload << 28) | ((uint32_t)abtpagetime << 16) | ((uint32_t)abtinqen << 15) | ((uint32_t)abtinqstartvalue << 14) | ((uint32_t)abtinqload << 12) | ((uint32_t)abtinqtime << 0));
}

__INLINE void bt_abtraincntl_unpack(uint8_t *abtpageen, uint8_t *abtpagestartvalue, uint8_t *abtpageload, uint16_t *abtpagetime, uint8_t *abtinqen, uint8_t *abtinqstartvalue, uint8_t *abtinqload, uint16_t *abtinqtime)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);

    *abtpageen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *abtpagestartvalue = (localVal & ((uint32_t)0x40000000)) >> 30;
    *abtpageload = (localVal & ((uint32_t)0x10000000)) >> 28;
    *abtpagetime = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *abtinqen = (localVal & ((uint32_t)0x00008000)) >> 15;
    *abtinqstartvalue = (localVal & ((uint32_t)0x00004000)) >> 14;
    *abtinqload = (localVal & ((uint32_t)0x00001000)) >> 12;
    *abtinqtime = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint8_t bt_abtraincntl_abtpageen_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_abtraincntl_abtpageen_setf(uint8_t abtpageen)
{
    ASSERT_ERR((((uint32_t)abtpageen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)abtpageen << 31));
}

__INLINE uint8_t bt_abtraincntl_abtpagestartvalue_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void bt_abtraincntl_abtpagestartvalue_setf(uint8_t abtpagestartvalue)
{
    ASSERT_ERR((((uint32_t)abtpagestartvalue << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)abtpagestartvalue << 30));
}

__INLINE uint8_t bt_abtraincntl_abtpageload_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void bt_abtraincntl_abtpageload_setf(uint8_t abtpageload)
{
    ASSERT_ERR((((uint32_t)abtpageload << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)abtpageload << 28));
}

__INLINE uint16_t bt_abtraincntl_abtpagetime_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE void bt_abtraincntl_abtpagetime_setf(uint16_t abtpagetime)
{
    ASSERT_ERR((((uint32_t)abtpagetime << 16) & ~((uint32_t)0x07FF0000)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x07FF0000)) | ((uint32_t)abtpagetime << 16));
}

__INLINE uint8_t bt_abtraincntl_abtinqen_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_abtraincntl_abtinqen_setf(uint8_t abtinqen)
{
    ASSERT_ERR((((uint32_t)abtinqen << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)abtinqen << 15));
}

__INLINE uint8_t bt_abtraincntl_abtinqstartvalue_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_abtraincntl_abtinqstartvalue_setf(uint8_t abtinqstartvalue)
{
    ASSERT_ERR((((uint32_t)abtinqstartvalue << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)abtinqstartvalue << 14));
}

__INLINE uint8_t bt_abtraincntl_abtinqload_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_abtraincntl_abtinqload_setf(uint8_t abtinqload)
{
    ASSERT_ERR((((uint32_t)abtinqload << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)abtinqload << 12));
}

__INLINE uint16_t bt_abtraincntl_abtinqtime_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_ABTRAINCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

__INLINE void bt_abtraincntl_abtinqtime_setf(uint16_t abtinqtime)
{
    ASSERT_ERR((((uint32_t)abtinqtime << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_ABTRAINCNTL_ADDR, (REG_BT_RD(BT_ABTRAINCNTL_ADDR) & ~((uint32_t)0x000007FF)) | ((uint32_t)abtinqtime << 0));
}

/**
 * @brief EDRCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16     TXRATE_SWINSTANT   0
 *     15             EDRBCAST   0
 *     13              RX_SWAP   0
 *     12              TX_SWAP   0
 *  10:08      GUARD_BAND_TIME   0x5
 *     07           RXGUARDDSB   0
 *     06    GB_TXQUAL_GEN_DSB   0
 *  05:00        RXGRD_TIMEOUT   0x12
 * </pre>
 */
#define BT_EDRCNTL_ADDR   0x50090534
#define BT_EDRCNTL_OFFSET 0x00000134
#define BT_EDRCNTL_INDEX  0x0000004D
#define BT_EDRCNTL_RESET  0x00000512

__INLINE uint32_t bt_edrcntl_get(void)
{
    return REG_BT_RD(BT_EDRCNTL_ADDR);
}

__INLINE void bt_edrcntl_set(uint32_t value)
{
    REG_BT_WR(BT_EDRCNTL_ADDR, value);
}

// field definitions
#define BT_TXRATE_SWINSTANT_BIT     ((uint32_t)0x00010000)
#define BT_TXRATE_SWINSTANT_POS     16
#define BT_EDRBCAST_BIT             ((uint32_t)0x00008000)
#define BT_EDRBCAST_POS             15
#define BT_RX_SWAP_BIT              ((uint32_t)0x00002000)
#define BT_RX_SWAP_POS              13
#define BT_TX_SWAP_BIT              ((uint32_t)0x00001000)
#define BT_TX_SWAP_POS              12
#define BT_GUARD_BAND_TIME_MASK     ((uint32_t)0x00000700)
#define BT_GUARD_BAND_TIME_LSB      8
#define BT_GUARD_BAND_TIME_WIDTH    ((uint32_t)0x00000003)
#define BT_RXGUARDDSB_BIT           ((uint32_t)0x00000080)
#define BT_RXGUARDDSB_POS           7
#define BT_GB_TXQUAL_GEN_DSB_BIT    ((uint32_t)0x00000040)
#define BT_GB_TXQUAL_GEN_DSB_POS    6
#define BT_RXGRD_TIMEOUT_MASK       ((uint32_t)0x0000003F)
#define BT_RXGRD_TIMEOUT_LSB        0
#define BT_RXGRD_TIMEOUT_WIDTH      ((uint32_t)0x00000006)

#define BT_TXRATE_SWINSTANT_RST     0x0
#define BT_EDRBCAST_RST             0x0
#define BT_RX_SWAP_RST              0x0
#define BT_TX_SWAP_RST              0x0
#define BT_GUARD_BAND_TIME_RST      0x5
#define BT_RXGUARDDSB_RST           0x0
#define BT_GB_TXQUAL_GEN_DSB_RST    0x0
#define BT_RXGRD_TIMEOUT_RST        0x12

__INLINE void bt_edrcntl_pack(uint8_t txrateswinstant, uint8_t edrbcast, uint8_t rxswap, uint8_t txswap, uint8_t guardbandtime, uint8_t rxguarddsb, uint8_t gbtxqualgendsb, uint8_t rxgrdtimeout)
{
    ASSERT_ERR((((uint32_t)txrateswinstant << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)edrbcast << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)rxswap << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)txswap << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)guardbandtime << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)rxguarddsb << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)gbtxqualgendsb << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)rxgrdtimeout << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, ((uint32_t)txrateswinstant << 16) | ((uint32_t)edrbcast << 15) | ((uint32_t)rxswap << 13) | ((uint32_t)txswap << 12) | ((uint32_t)guardbandtime << 8) | ((uint32_t)rxguarddsb << 7) | ((uint32_t)gbtxqualgendsb << 6) | ((uint32_t)rxgrdtimeout << 0));
}

__INLINE void bt_edrcntl_unpack(uint8_t *txrateswinstant, uint8_t *edrbcast, uint8_t *rxswap, uint8_t *txswap, uint8_t *guardbandtime, uint8_t *rxguarddsb, uint8_t *gbtxqualgendsb, uint8_t *rxgrdtimeout)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);

    *txrateswinstant = (localVal & ((uint32_t)0x00010000)) >> 16;
    *edrbcast = (localVal & ((uint32_t)0x00008000)) >> 15;
    *rxswap = (localVal & ((uint32_t)0x00002000)) >> 13;
    *txswap = (localVal & ((uint32_t)0x00001000)) >> 12;
    *guardbandtime = (localVal & ((uint32_t)0x00000700)) >> 8;
    *rxguarddsb = (localVal & ((uint32_t)0x00000080)) >> 7;
    *gbtxqualgendsb = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxgrdtimeout = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t bt_edrcntl_txrate_swinstant_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bt_edrcntl_txrate_swinstant_setf(uint8_t txrateswinstant)
{
    ASSERT_ERR((((uint32_t)txrateswinstant << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)txrateswinstant << 16));
}

__INLINE uint8_t bt_edrcntl_edrbcast_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_edrcntl_edrbcast_setf(uint8_t edrbcast)
{
    ASSERT_ERR((((uint32_t)edrbcast << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)edrbcast << 15));
}

__INLINE uint8_t bt_edrcntl_rx_swap_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_edrcntl_rx_swap_setf(uint8_t rxswap)
{
    ASSERT_ERR((((uint32_t)rxswap << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)rxswap << 13));
}

__INLINE uint8_t bt_edrcntl_tx_swap_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_edrcntl_tx_swap_setf(uint8_t txswap)
{
    ASSERT_ERR((((uint32_t)txswap << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)txswap << 12));
}

__INLINE uint8_t bt_edrcntl_guard_band_time_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void bt_edrcntl_guard_band_time_setf(uint8_t guardbandtime)
{
    ASSERT_ERR((((uint32_t)guardbandtime << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)guardbandtime << 8));
}

__INLINE uint8_t bt_edrcntl_rxguarddsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_edrcntl_rxguarddsb_setf(uint8_t rxguarddsb)
{
    ASSERT_ERR((((uint32_t)rxguarddsb << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rxguarddsb << 7));
}

__INLINE uint8_t bt_edrcntl_gb_txqual_gen_dsb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bt_edrcntl_gb_txqual_gen_dsb_setf(uint8_t gbtxqualgendsb)
{
    ASSERT_ERR((((uint32_t)gbtxqualgendsb << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)gbtxqualgendsb << 6));
}

__INLINE uint8_t bt_edrcntl_rxgrd_timeout_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_EDRCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

__INLINE void bt_edrcntl_rxgrd_timeout_setf(uint8_t rxgrdtimeout)
{
    ASSERT_ERR((((uint32_t)rxgrdtimeout << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_BT_WR(BT_EDRCNTL_ADDR, (REG_BT_RD(BT_EDRCNTL_ADDR) & ~((uint32_t)0x0000003F)) | ((uint32_t)rxgrdtimeout << 0));
}

/**
 * @brief PCACNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16        TARGET_OFFSET   0x0
 *  12:08               SLVLBL   0x0
 *  07:04            CORR_STEP   0x1
 *     03         BLINDCORR_EN   0
 *     02           FRSYNC_POL   0
 *     01          SYNC_SOURCE   0
 *     00       PHASE_SHIFT_EN   0
 * </pre>
 */
#define BT_PCACNTL0_ADDR   0x50090540
#define BT_PCACNTL0_OFFSET 0x00000140
#define BT_PCACNTL0_INDEX  0x00000050
#define BT_PCACNTL0_RESET  0x00000010

__INLINE uint32_t bt_pcacntl0_get(void)
{
    return REG_BT_RD(BT_PCACNTL0_ADDR);
}

__INLINE void bt_pcacntl0_set(uint32_t value)
{
    REG_BT_WR(BT_PCACNTL0_ADDR, value);
}

// field definitions
#define BT_TARGET_OFFSET_MASK    ((uint32_t)0x07FF0000)
#define BT_TARGET_OFFSET_LSB     16
#define BT_TARGET_OFFSET_WIDTH   ((uint32_t)0x0000000B)
#define BT_SLVLBL_MASK           ((uint32_t)0x00001F00)
#define BT_SLVLBL_LSB            8
#define BT_SLVLBL_WIDTH          ((uint32_t)0x00000005)
#define BT_CORR_STEP_MASK        ((uint32_t)0x000000F0)
#define BT_CORR_STEP_LSB         4
#define BT_CORR_STEP_WIDTH       ((uint32_t)0x00000004)
#define BT_BLINDCORR_EN_BIT      ((uint32_t)0x00000008)
#define BT_BLINDCORR_EN_POS      3
#define BT_FRSYNC_POL_BIT        ((uint32_t)0x00000004)
#define BT_FRSYNC_POL_POS        2
#define BT_SYNC_SOURCE_BIT       ((uint32_t)0x00000002)
#define BT_SYNC_SOURCE_POS       1
#define BT_PHASE_SHIFT_EN_BIT    ((uint32_t)0x00000001)
#define BT_PHASE_SHIFT_EN_POS    0

#define BT_TARGET_OFFSET_RST     0x0
#define BT_SLVLBL_RST            0x0
#define BT_CORR_STEP_RST         0x1
#define BT_BLINDCORR_EN_RST      0x0
#define BT_FRSYNC_POL_RST        0x0
#define BT_SYNC_SOURCE_RST       0x0
#define BT_PHASE_SHIFT_EN_RST    0x0

__INLINE void bt_pcacntl0_pack(uint16_t targetoffset, uint8_t slvlbl, uint8_t corrstep, uint8_t blindcorren, uint8_t frsyncpol, uint8_t syncsource, uint8_t phaseshiften)
{
    ASSERT_ERR((((uint32_t)targetoffset << 16) & ~((uint32_t)0x07FF0000)) == 0);
    ASSERT_ERR((((uint32_t)slvlbl << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)corrstep << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)blindcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)frsyncpol << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)syncsource << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)phaseshiften << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, ((uint32_t)targetoffset << 16) | ((uint32_t)slvlbl << 8) | ((uint32_t)corrstep << 4) | ((uint32_t)blindcorren << 3) | ((uint32_t)frsyncpol << 2) | ((uint32_t)syncsource << 1) | ((uint32_t)phaseshiften << 0));
}

__INLINE void bt_pcacntl0_unpack(uint16_t *targetoffset, uint8_t *slvlbl, uint8_t *corrstep, uint8_t *blindcorren, uint8_t *frsyncpol, uint8_t *syncsource, uint8_t *phaseshiften)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);

    *targetoffset = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *slvlbl = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *corrstep = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *blindcorren = (localVal & ((uint32_t)0x00000008)) >> 3;
    *frsyncpol = (localVal & ((uint32_t)0x00000004)) >> 2;
    *syncsource = (localVal & ((uint32_t)0x00000002)) >> 1;
    *phaseshiften = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t bt_pcacntl0_target_offset_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE void bt_pcacntl0_target_offset_setf(uint16_t targetoffset)
{
    ASSERT_ERR((((uint32_t)targetoffset << 16) & ~((uint32_t)0x07FF0000)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, (REG_BT_RD(BT_PCACNTL0_ADDR) & ~((uint32_t)0x07FF0000)) | ((uint32_t)targetoffset << 16));
}

__INLINE uint8_t bt_pcacntl0_slvlbl_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void bt_pcacntl0_slvlbl_setf(uint8_t slvlbl)
{
    ASSERT_ERR((((uint32_t)slvlbl << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, (REG_BT_RD(BT_PCACNTL0_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)slvlbl << 8));
}

__INLINE uint8_t bt_pcacntl0_corr_step_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bt_pcacntl0_corr_step_setf(uint8_t corrstep)
{
    ASSERT_ERR((((uint32_t)corrstep << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, (REG_BT_RD(BT_PCACNTL0_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)corrstep << 4));
}

__INLINE uint8_t bt_pcacntl0_blindcorr_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_pcacntl0_blindcorr_en_setf(uint8_t blindcorren)
{
    ASSERT_ERR((((uint32_t)blindcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, (REG_BT_RD(BT_PCACNTL0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)blindcorren << 3));
}

__INLINE uint8_t bt_pcacntl0_frsync_pol_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_pcacntl0_frsync_pol_setf(uint8_t frsyncpol)
{
    ASSERT_ERR((((uint32_t)frsyncpol << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, (REG_BT_RD(BT_PCACNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)frsyncpol << 2));
}

__INLINE uint8_t bt_pcacntl0_sync_source_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_pcacntl0_sync_source_setf(uint8_t syncsource)
{
    ASSERT_ERR((((uint32_t)syncsource << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, (REG_BT_RD(BT_PCACNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)syncsource << 1));
}

__INLINE uint8_t bt_pcacntl0_phase_shift_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_pcacntl0_phase_shift_en_setf(uint8_t phaseshiften)
{
    ASSERT_ERR((((uint32_t)phaseshiften << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_PCACNTL0_ADDR, (REG_BT_RD(BT_PCACNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)phaseshiften << 0));
}

/**
 * @brief PCACNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16        CORR_INTERVAL   0x28
 *     12       CLOCK_SHIFT_EN   0
 *  10:00          CLOCK_SHIFT   0x0
 * </pre>
 */
#define BT_PCACNTL1_ADDR   0x50090544
#define BT_PCACNTL1_OFFSET 0x00000144
#define BT_PCACNTL1_INDEX  0x00000051
#define BT_PCACNTL1_RESET  0x00280000

__INLINE uint32_t bt_pcacntl1_get(void)
{
    return REG_BT_RD(BT_PCACNTL1_ADDR);
}

__INLINE void bt_pcacntl1_set(uint32_t value)
{
    REG_BT_WR(BT_PCACNTL1_ADDR, value);
}

// field definitions
#define BT_CORR_INTERVAL_MASK    ((uint32_t)0x00FF0000)
#define BT_CORR_INTERVAL_LSB     16
#define BT_CORR_INTERVAL_WIDTH   ((uint32_t)0x00000008)
#define BT_CLOCK_SHIFT_EN_BIT    ((uint32_t)0x00001000)
#define BT_CLOCK_SHIFT_EN_POS    12
#define BT_CLOCK_SHIFT_MASK      ((uint32_t)0x000007FF)
#define BT_CLOCK_SHIFT_LSB       0
#define BT_CLOCK_SHIFT_WIDTH     ((uint32_t)0x0000000B)

#define BT_CORR_INTERVAL_RST     0x28
#define BT_CLOCK_SHIFT_EN_RST    0x0
#define BT_CLOCK_SHIFT_RST       0x0

__INLINE void bt_pcacntl1_pack(uint8_t corrinterval, uint8_t clockshiften, uint16_t clockshift)
{
    ASSERT_ERR((((uint32_t)corrinterval << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)clockshiften << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)clockshift << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_PCACNTL1_ADDR, ((uint32_t)corrinterval << 16) | ((uint32_t)clockshiften << 12) | ((uint32_t)clockshift << 0));
}

__INLINE void bt_pcacntl1_unpack(uint8_t *corrinterval, uint8_t *clockshiften, uint16_t *clockshift)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL1_ADDR);

    *corrinterval = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *clockshiften = (localVal & ((uint32_t)0x00001000)) >> 12;
    *clockshift = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint8_t bt_pcacntl1_corr_interval_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bt_pcacntl1_corr_interval_setf(uint8_t corrinterval)
{
    ASSERT_ERR((((uint32_t)corrinterval << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(BT_PCACNTL1_ADDR, (REG_BT_RD(BT_PCACNTL1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)corrinterval << 16));
}

__INLINE uint8_t bt_pcacntl1_clock_shift_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_pcacntl1_clock_shift_en_setf(uint8_t clockshiften)
{
    ASSERT_ERR((((uint32_t)clockshiften << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_PCACNTL1_ADDR, (REG_BT_RD(BT_PCACNTL1_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)clockshiften << 12));
}

__INLINE uint16_t bt_pcacntl1_clock_shift_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCACNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

__INLINE void bt_pcacntl1_clock_shift_setf(uint16_t clockshift)
{
    ASSERT_ERR((((uint32_t)clockshift << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_PCACNTL1_ADDR, (REG_BT_RD(BT_PCACNTL1_ADDR) & ~((uint32_t)0x000007FF)) | ((uint32_t)clockshift << 0));
}

/**
 * @brief PCASTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16          SHIFT_PHASE   0x0
 *  10:00        MOMENT_OFFSET   0x0
 * </pre>
 */
#define BT_PCASTAT_ADDR   0x50090548
#define BT_PCASTAT_OFFSET 0x00000148
#define BT_PCASTAT_INDEX  0x00000052
#define BT_PCASTAT_RESET  0x00000000

__INLINE uint32_t bt_pcastat_get(void)
{
    return REG_BT_RD(BT_PCASTAT_ADDR);
}

// field definitions
#define BT_SHIFT_PHASE_MASK     ((uint32_t)0x07FF0000)
#define BT_SHIFT_PHASE_LSB      16
#define BT_SHIFT_PHASE_WIDTH    ((uint32_t)0x0000000B)
#define BT_MOMENT_OFFSET_MASK   ((uint32_t)0x000007FF)
#define BT_MOMENT_OFFSET_LSB    0
#define BT_MOMENT_OFFSET_WIDTH  ((uint32_t)0x0000000B)

#define BT_SHIFT_PHASE_RST      0x0
#define BT_MOMENT_OFFSET_RST    0x0

__INLINE void bt_pcastat_unpack(uint16_t *shiftphase, uint16_t *momentoffset)
{
    uint32_t localVal = REG_BT_RD(BT_PCASTAT_ADDR);

    *shiftphase = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *momentoffset = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint16_t bt_pcastat_shift_phase_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCASTAT_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE uint16_t bt_pcastat_moment_offset_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCASTAT_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

/**
 * @brief COEXIFCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28         INQKNUDGEINC   0x2
 *  27:24        PAGEKNUDGEINC   0x2
 *  21:20       MWSSCANFREQMSK   0x0
 *  19:18        WLCRXPRIOMODE   0x0
 *  17:16        WLCTXPRIOMODE   0x0
 *  15:14          MWSTXFRQMSK   0x0
 *  13:12          MWSRXFRQMSK   0x0
 *  11:10             MWSTXMSK   0x0
 *  09:08             MWSRXMSK   0x0
 *  07:06            WLANTXMSK   0x0
 *  05:04            WLANRXMSK   0x1
 *     03            MWSWCI_EN   0
 *     02           MWSCOEX_EN   0
 *     01           SYNCGEN_EN   0
 *     00          WLANCOEX_EN   0
 * </pre>
 */
#define BT_COEXIFCNTL0_ADDR   0x50090550
#define BT_COEXIFCNTL0_OFFSET 0x00000150
#define BT_COEXIFCNTL0_INDEX  0x00000054
#define BT_COEXIFCNTL0_RESET  0x22000010

__INLINE uint32_t bt_coexifcntl0_get(void)
{
    return REG_BT_RD(BT_COEXIFCNTL0_ADDR);
}

__INLINE void bt_coexifcntl0_set(uint32_t value)
{
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, value);
}

// field definitions
#define BT_INQKNUDGEINC_MASK     ((uint32_t)0xF0000000)
#define BT_INQKNUDGEINC_LSB      28
#define BT_INQKNUDGEINC_WIDTH    ((uint32_t)0x00000004)
#define BT_PAGEKNUDGEINC_MASK    ((uint32_t)0x0F000000)
#define BT_PAGEKNUDGEINC_LSB     24
#define BT_PAGEKNUDGEINC_WIDTH   ((uint32_t)0x00000004)
#define BT_MWSSCANFREQMSK_MASK   ((uint32_t)0x00300000)
#define BT_MWSSCANFREQMSK_LSB    20
#define BT_MWSSCANFREQMSK_WIDTH  ((uint32_t)0x00000002)
#define BT_WLCRXPRIOMODE_MASK    ((uint32_t)0x000C0000)
#define BT_WLCRXPRIOMODE_LSB     18
#define BT_WLCRXPRIOMODE_WIDTH   ((uint32_t)0x00000002)
#define BT_WLCTXPRIOMODE_MASK    ((uint32_t)0x00030000)
#define BT_WLCTXPRIOMODE_LSB     16
#define BT_WLCTXPRIOMODE_WIDTH   ((uint32_t)0x00000002)
#define BT_MWSTXFRQMSK_MASK      ((uint32_t)0x0000C000)
#define BT_MWSTXFRQMSK_LSB       14
#define BT_MWSTXFRQMSK_WIDTH     ((uint32_t)0x00000002)
#define BT_MWSRXFRQMSK_MASK      ((uint32_t)0x00003000)
#define BT_MWSRXFRQMSK_LSB       12
#define BT_MWSRXFRQMSK_WIDTH     ((uint32_t)0x00000002)
#define BT_MWSTXMSK_MASK         ((uint32_t)0x00000C00)
#define BT_MWSTXMSK_LSB          10
#define BT_MWSTXMSK_WIDTH        ((uint32_t)0x00000002)
#define BT_MWSRXMSK_MASK         ((uint32_t)0x00000300)
#define BT_MWSRXMSK_LSB          8
#define BT_MWSRXMSK_WIDTH        ((uint32_t)0x00000002)
#define BT_WLANTXMSK_MASK        ((uint32_t)0x000000C0)
#define BT_WLANTXMSK_LSB         6
#define BT_WLANTXMSK_WIDTH       ((uint32_t)0x00000002)
#define BT_WLANRXMSK_MASK        ((uint32_t)0x00000030)
#define BT_WLANRXMSK_LSB         4
#define BT_WLANRXMSK_WIDTH       ((uint32_t)0x00000002)
#define BT_MWSWCI_EN_BIT         ((uint32_t)0x00000008)
#define BT_MWSWCI_EN_POS         3
#define BT_MWSCOEX_EN_BIT        ((uint32_t)0x00000004)
#define BT_MWSCOEX_EN_POS        2
#define BT_SYNCGEN_EN_BIT        ((uint32_t)0x00000002)
#define BT_SYNCGEN_EN_POS        1
#define BT_WLANCOEX_EN_BIT       ((uint32_t)0x00000001)
#define BT_WLANCOEX_EN_POS       0

#define BT_INQKNUDGEINC_RST      0x2
#define BT_PAGEKNUDGEINC_RST     0x2
#define BT_MWSSCANFREQMSK_RST    0x0
#define BT_WLCRXPRIOMODE_RST     0x0
#define BT_WLCTXPRIOMODE_RST     0x0
#define BT_MWSTXFRQMSK_RST       0x0
#define BT_MWSRXFRQMSK_RST       0x0
#define BT_MWSTXMSK_RST          0x0
#define BT_MWSRXMSK_RST          0x0
#define BT_WLANTXMSK_RST         0x0
#define BT_WLANRXMSK_RST         0x1
#define BT_MWSWCI_EN_RST         0x0
#define BT_MWSCOEX_EN_RST        0x0
#define BT_SYNCGEN_EN_RST        0x0
#define BT_WLANCOEX_EN_RST       0x0

__INLINE void bt_coexifcntl0_pack(uint8_t inqknudgeinc, uint8_t pageknudgeinc, uint8_t mwsscanfreqmsk, uint8_t wlcrxpriomode, uint8_t wlctxpriomode, uint8_t mwstxfrqmsk, uint8_t mwsrxfrqmsk, uint8_t mwstxmsk, uint8_t mwsrxmsk, uint8_t wlantxmsk, uint8_t wlanrxmsk, uint8_t mwswcien, uint8_t mwscoexen, uint8_t syncgenen, uint8_t wlancoexen)
{
    ASSERT_ERR((((uint32_t)inqknudgeinc << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)pageknudgeinc << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreqmsk << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)wlcrxpriomode << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)wlctxpriomode << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mwstxfrqmsk << 14) & ~((uint32_t)0x0000C000)) == 0);
    ASSERT_ERR((((uint32_t)mwsrxfrqmsk << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)mwstxmsk << 10) & ~((uint32_t)0x00000C00)) == 0);
    ASSERT_ERR((((uint32_t)mwsrxmsk << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)wlantxmsk << 6) & ~((uint32_t)0x000000C0)) == 0);
    ASSERT_ERR((((uint32_t)wlanrxmsk << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)mwswcien << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)mwscoexen << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)syncgenen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)wlancoexen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, ((uint32_t)inqknudgeinc << 28) | ((uint32_t)pageknudgeinc << 24) | ((uint32_t)mwsscanfreqmsk << 20) | ((uint32_t)wlcrxpriomode << 18) | ((uint32_t)wlctxpriomode << 16) | ((uint32_t)mwstxfrqmsk << 14) | ((uint32_t)mwsrxfrqmsk << 12) | ((uint32_t)mwstxmsk << 10) | ((uint32_t)mwsrxmsk << 8) | ((uint32_t)wlantxmsk << 6) | ((uint32_t)wlanrxmsk << 4) | ((uint32_t)mwswcien << 3) | ((uint32_t)mwscoexen << 2) | ((uint32_t)syncgenen << 1) | ((uint32_t)wlancoexen << 0));
}

__INLINE void bt_coexifcntl0_unpack(uint8_t *inqknudgeinc, uint8_t *pageknudgeinc, uint8_t *mwsscanfreqmsk, uint8_t *wlcrxpriomode, uint8_t *wlctxpriomode, uint8_t *mwstxfrqmsk, uint8_t *mwsrxfrqmsk, uint8_t *mwstxmsk, uint8_t *mwsrxmsk, uint8_t *wlantxmsk, uint8_t *wlanrxmsk, uint8_t *mwswcien, uint8_t *mwscoexen, uint8_t *syncgenen, uint8_t *wlancoexen)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);

    *inqknudgeinc = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *pageknudgeinc = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *mwsscanfreqmsk = (localVal & ((uint32_t)0x00300000)) >> 20;
    *wlcrxpriomode = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *wlctxpriomode = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mwstxfrqmsk = (localVal & ((uint32_t)0x0000C000)) >> 14;
    *mwsrxfrqmsk = (localVal & ((uint32_t)0x00003000)) >> 12;
    *mwstxmsk = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *mwsrxmsk = (localVal & ((uint32_t)0x00000300)) >> 8;
    *wlantxmsk = (localVal & ((uint32_t)0x000000C0)) >> 6;
    *wlanrxmsk = (localVal & ((uint32_t)0x00000030)) >> 4;
    *mwswcien = (localVal & ((uint32_t)0x00000008)) >> 3;
    *mwscoexen = (localVal & ((uint32_t)0x00000004)) >> 2;
    *syncgenen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *wlancoexen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_coexifcntl0_inqknudgeinc_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void bt_coexifcntl0_inqknudgeinc_setf(uint8_t inqknudgeinc)
{
    ASSERT_ERR((((uint32_t)inqknudgeinc << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)inqknudgeinc << 28));
}

__INLINE uint8_t bt_coexifcntl0_pageknudgeinc_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void bt_coexifcntl0_pageknudgeinc_setf(uint8_t pageknudgeinc)
{
    ASSERT_ERR((((uint32_t)pageknudgeinc << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)pageknudgeinc << 24));
}

__INLINE uint8_t bt_coexifcntl0_mwsscanfreqmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void bt_coexifcntl0_mwsscanfreqmsk_setf(uint8_t mwsscanfreqmsk)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqmsk << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)mwsscanfreqmsk << 20));
}

__INLINE uint8_t bt_coexifcntl0_wlcrxpriomode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void bt_coexifcntl0_wlcrxpriomode_setf(uint8_t wlcrxpriomode)
{
    ASSERT_ERR((((uint32_t)wlcrxpriomode << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)wlcrxpriomode << 18));
}

__INLINE uint8_t bt_coexifcntl0_wlctxpriomode_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_coexifcntl0_wlctxpriomode_setf(uint8_t wlctxpriomode)
{
    ASSERT_ERR((((uint32_t)wlctxpriomode << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)wlctxpriomode << 16));
}

__INLINE uint8_t bt_coexifcntl0_mwstxfrqmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void bt_coexifcntl0_mwstxfrqmsk_setf(uint8_t mwstxfrqmsk)
{
    ASSERT_ERR((((uint32_t)mwstxfrqmsk << 14) & ~((uint32_t)0x0000C000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)mwstxfrqmsk << 14));
}

__INLINE uint8_t bt_coexifcntl0_mwsrxfrqmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void bt_coexifcntl0_mwsrxfrqmsk_setf(uint8_t mwsrxfrqmsk)
{
    ASSERT_ERR((((uint32_t)mwsrxfrqmsk << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)mwsrxfrqmsk << 12));
}

__INLINE uint8_t bt_coexifcntl0_mwstxmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void bt_coexifcntl0_mwstxmsk_setf(uint8_t mwstxmsk)
{
    ASSERT_ERR((((uint32_t)mwstxmsk << 10) & ~((uint32_t)0x00000C00)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)mwstxmsk << 10));
}

__INLINE uint8_t bt_coexifcntl0_mwsrxmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void bt_coexifcntl0_mwsrxmsk_setf(uint8_t mwsrxmsk)
{
    ASSERT_ERR((((uint32_t)mwsrxmsk << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)mwsrxmsk << 8));
}

__INLINE uint8_t bt_coexifcntl0_wlantxmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000C0)) >> 6);
}

__INLINE void bt_coexifcntl0_wlantxmsk_setf(uint8_t wlantxmsk)
{
    ASSERT_ERR((((uint32_t)wlantxmsk << 6) & ~((uint32_t)0x000000C0)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x000000C0)) | ((uint32_t)wlantxmsk << 6));
}

__INLINE uint8_t bt_coexifcntl0_wlanrxmsk_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void bt_coexifcntl0_wlanrxmsk_setf(uint8_t wlanrxmsk)
{
    ASSERT_ERR((((uint32_t)wlanrxmsk << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)wlanrxmsk << 4));
}

__INLINE uint8_t bt_coexifcntl0_mwswci_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_coexifcntl0_mwswci_en_setf(uint8_t mwswcien)
{
    ASSERT_ERR((((uint32_t)mwswcien << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)mwswcien << 3));
}

__INLINE uint8_t bt_coexifcntl0_mwscoex_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_coexifcntl0_mwscoex_en_setf(uint8_t mwscoexen)
{
    ASSERT_ERR((((uint32_t)mwscoexen << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)mwscoexen << 2));
}

__INLINE uint8_t bt_coexifcntl0_syncgen_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_coexifcntl0_syncgen_en_setf(uint8_t syncgenen)
{
    ASSERT_ERR((((uint32_t)syncgenen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)syncgenen << 1));
}

__INLINE uint8_t bt_coexifcntl0_wlancoex_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_coexifcntl0_wlancoex_en_setf(uint8_t wlancoexen)
{
    ASSERT_ERR((((uint32_t)wlancoexen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_COEXIFCNTL0_ADDR, (REG_BT_RD(BT_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)wlancoexen << 0));
}

/**
 * @brief COEXIFCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:24            WLCPRXTHR   0x0
 *  20:16            WLCPTXTHR   0x0
 *  14:08         WLCPDURATION   0x0
 *  06:00            WLCPDELAY   0x0
 * </pre>
 */
#define BT_COEXIFCNTL1_ADDR   0x50090554
#define BT_COEXIFCNTL1_OFFSET 0x00000154
#define BT_COEXIFCNTL1_INDEX  0x00000055
#define BT_COEXIFCNTL1_RESET  0x00000000

__INLINE uint32_t bt_coexifcntl1_get(void)
{
    return REG_BT_RD(BT_COEXIFCNTL1_ADDR);
}

__INLINE void bt_coexifcntl1_set(uint32_t value)
{
    REG_BT_WR(BT_COEXIFCNTL1_ADDR, value);
}

// field definitions
#define BT_WLCPRXTHR_MASK      ((uint32_t)0x1F000000)
#define BT_WLCPRXTHR_LSB       24
#define BT_WLCPRXTHR_WIDTH     ((uint32_t)0x00000005)
#define BT_WLCPTXTHR_MASK      ((uint32_t)0x001F0000)
#define BT_WLCPTXTHR_LSB       16
#define BT_WLCPTXTHR_WIDTH     ((uint32_t)0x00000005)
#define BT_WLCPDURATION_MASK   ((uint32_t)0x00007F00)
#define BT_WLCPDURATION_LSB    8
#define BT_WLCPDURATION_WIDTH  ((uint32_t)0x00000007)
#define BT_WLCPDELAY_MASK      ((uint32_t)0x0000007F)
#define BT_WLCPDELAY_LSB       0
#define BT_WLCPDELAY_WIDTH     ((uint32_t)0x00000007)

#define BT_WLCPRXTHR_RST       0x0
#define BT_WLCPTXTHR_RST       0x0
#define BT_WLCPDURATION_RST    0x0
#define BT_WLCPDELAY_RST       0x0

__INLINE void bt_coexifcntl1_pack(uint8_t wlcprxthr, uint8_t wlcptxthr, uint8_t wlcpduration, uint8_t wlcpdelay)
{
    ASSERT_ERR((((uint32_t)wlcprxthr << 24) & ~((uint32_t)0x1F000000)) == 0);
    ASSERT_ERR((((uint32_t)wlcptxthr << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)wlcpduration << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)wlcpdelay << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_COEXIFCNTL1_ADDR, ((uint32_t)wlcprxthr << 24) | ((uint32_t)wlcptxthr << 16) | ((uint32_t)wlcpduration << 8) | ((uint32_t)wlcpdelay << 0));
}

__INLINE void bt_coexifcntl1_unpack(uint8_t *wlcprxthr, uint8_t *wlcptxthr, uint8_t *wlcpduration, uint8_t *wlcpdelay)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL1_ADDR);

    *wlcprxthr = (localVal & ((uint32_t)0x1F000000)) >> 24;
    *wlcptxthr = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *wlcpduration = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *wlcpdelay = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t bt_coexifcntl1_wlcprxthr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x1F000000)) >> 24);
}

__INLINE void bt_coexifcntl1_wlcprxthr_setf(uint8_t wlcprxthr)
{
    ASSERT_ERR((((uint32_t)wlcprxthr << 24) & ~((uint32_t)0x1F000000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL1_ADDR, (REG_BT_RD(BT_COEXIFCNTL1_ADDR) & ~((uint32_t)0x1F000000)) | ((uint32_t)wlcprxthr << 24));
}

__INLINE uint8_t bt_coexifcntl1_wlcptxthr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void bt_coexifcntl1_wlcptxthr_setf(uint8_t wlcptxthr)
{
    ASSERT_ERR((((uint32_t)wlcptxthr << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_BT_WR(BT_COEXIFCNTL1_ADDR, (REG_BT_RD(BT_COEXIFCNTL1_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)wlcptxthr << 16));
}

__INLINE uint8_t bt_coexifcntl1_wlcpduration_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void bt_coexifcntl1_wlcpduration_setf(uint8_t wlcpduration)
{
    ASSERT_ERR((((uint32_t)wlcpduration << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BT_WR(BT_COEXIFCNTL1_ADDR, (REG_BT_RD(BT_COEXIFCNTL1_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)wlcpduration << 8));
}

__INLINE uint8_t bt_coexifcntl1_wlcpdelay_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void bt_coexifcntl1_wlcpdelay_setf(uint8_t wlcpdelay)
{
    ASSERT_ERR((((uint32_t)wlcpdelay << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BT_WR(BT_COEXIFCNTL1_ADDR, (REG_BT_RD(BT_COEXIFCNTL1_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)wlcpdelay << 0));
}

/**
 * @brief COEXIFCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:08         RX_ANT_DELAY   0x0
 *  03:00         TX_ANT_DELAY   0x0
 * </pre>
 */
#define BT_COEXIFCNTL2_ADDR   0x50090558
#define BT_COEXIFCNTL2_OFFSET 0x00000158
#define BT_COEXIFCNTL2_INDEX  0x00000056
#define BT_COEXIFCNTL2_RESET  0x00000000

__INLINE uint32_t bt_coexifcntl2_get(void)
{
    return REG_BT_RD(BT_COEXIFCNTL2_ADDR);
}

__INLINE void bt_coexifcntl2_set(uint32_t value)
{
    REG_BT_WR(BT_COEXIFCNTL2_ADDR, value);
}

// field definitions
#define BT_RX_ANT_DELAY_MASK   ((uint32_t)0x00000F00)
#define BT_RX_ANT_DELAY_LSB    8
#define BT_RX_ANT_DELAY_WIDTH  ((uint32_t)0x00000004)
#define BT_TX_ANT_DELAY_MASK   ((uint32_t)0x0000000F)
#define BT_TX_ANT_DELAY_LSB    0
#define BT_TX_ANT_DELAY_WIDTH  ((uint32_t)0x00000004)

#define BT_RX_ANT_DELAY_RST    0x0
#define BT_TX_ANT_DELAY_RST    0x0

__INLINE void bt_coexifcntl2_pack(uint8_t rxantdelay, uint8_t txantdelay)
{
    ASSERT_ERR((((uint32_t)rxantdelay << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)txantdelay << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_COEXIFCNTL2_ADDR, ((uint32_t)rxantdelay << 8) | ((uint32_t)txantdelay << 0));
}

__INLINE void bt_coexifcntl2_unpack(uint8_t *rxantdelay, uint8_t *txantdelay)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL2_ADDR);

    *rxantdelay = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *txantdelay = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_coexifcntl2_rx_ant_delay_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_coexifcntl2_rx_ant_delay_setf(uint8_t rxantdelay)
{
    ASSERT_ERR((((uint32_t)rxantdelay << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_COEXIFCNTL2_ADDR, (REG_BT_RD(BT_COEXIFCNTL2_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)rxantdelay << 8));
}

__INLINE uint8_t bt_coexifcntl2_tx_ant_delay_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_COEXIFCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_coexifcntl2_tx_ant_delay_setf(uint8_t txantdelay)
{
    ASSERT_ERR((((uint32_t)txantdelay << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_COEXIFCNTL2_ADDR, (REG_BT_RD(BT_COEXIFCNTL2_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)txantdelay << 0));
}

/**
 * @brief BTMPRIO0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28                 BTM7   0xB
 *  27:24                 BTM6   0xC
 *  23:20                 BTM5   0xD
 *  19:16                 BTM4   0xE
 *  15:12                 BTM3   0xF
 *  11:08                 BTM2   0xF
 *  07:04                 BTM1   0xF
 *  03:00                 BTM0   0xF
 * </pre>
 */
#define BT_BTMPRIO0_ADDR   0x50090560
#define BT_BTMPRIO0_OFFSET 0x00000160
#define BT_BTMPRIO0_INDEX  0x00000058
#define BT_BTMPRIO0_RESET  0xBCDEFFFF

__INLINE uint32_t bt_btmprio0_get(void)
{
    return REG_BT_RD(BT_BTMPRIO0_ADDR);
}

__INLINE void bt_btmprio0_set(uint32_t value)
{
    REG_BT_WR(BT_BTMPRIO0_ADDR, value);
}

// field definitions
#define BT_BTM7_MASK   ((uint32_t)0xF0000000)
#define BT_BTM7_LSB    28
#define BT_BTM7_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM6_MASK   ((uint32_t)0x0F000000)
#define BT_BTM6_LSB    24
#define BT_BTM6_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM5_MASK   ((uint32_t)0x00F00000)
#define BT_BTM5_LSB    20
#define BT_BTM5_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM4_MASK   ((uint32_t)0x000F0000)
#define BT_BTM4_LSB    16
#define BT_BTM4_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM3_MASK   ((uint32_t)0x0000F000)
#define BT_BTM3_LSB    12
#define BT_BTM3_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM2_MASK   ((uint32_t)0x00000F00)
#define BT_BTM2_LSB    8
#define BT_BTM2_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM1_MASK   ((uint32_t)0x000000F0)
#define BT_BTM1_LSB    4
#define BT_BTM1_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM0_MASK   ((uint32_t)0x0000000F)
#define BT_BTM0_LSB    0
#define BT_BTM0_WIDTH  ((uint32_t)0x00000004)

#define BT_BTM7_RST    0xB
#define BT_BTM6_RST    0xC
#define BT_BTM5_RST    0xD
#define BT_BTM4_RST    0xE
#define BT_BTM3_RST    0xF
#define BT_BTM2_RST    0xF
#define BT_BTM1_RST    0xF
#define BT_BTM0_RST    0xF

__INLINE void bt_btmprio0_pack(uint8_t btm7, uint8_t btm6, uint8_t btm5, uint8_t btm4, uint8_t btm3, uint8_t btm2, uint8_t btm1, uint8_t btm0)
{
    ASSERT_ERR((((uint32_t)btm7 << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)btm6 << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)btm5 << 20) & ~((uint32_t)0x00F00000)) == 0);
    ASSERT_ERR((((uint32_t)btm4 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)btm3 << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)btm2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)btm1 << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)btm0 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, ((uint32_t)btm7 << 28) | ((uint32_t)btm6 << 24) | ((uint32_t)btm5 << 20) | ((uint32_t)btm4 << 16) | ((uint32_t)btm3 << 12) | ((uint32_t)btm2 << 8) | ((uint32_t)btm1 << 4) | ((uint32_t)btm0 << 0));
}

__INLINE void bt_btmprio0_unpack(uint8_t *btm7, uint8_t *btm6, uint8_t *btm5, uint8_t *btm4, uint8_t *btm3, uint8_t *btm2, uint8_t *btm1, uint8_t *btm0)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);

    *btm7 = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *btm6 = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *btm5 = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *btm4 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *btm3 = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *btm2 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *btm1 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *btm0 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_btmprio0_btm7_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void bt_btmprio0_btm7_setf(uint8_t btm7)
{
    ASSERT_ERR((((uint32_t)btm7 << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)btm7 << 28));
}

__INLINE uint8_t bt_btmprio0_btm6_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void bt_btmprio0_btm6_setf(uint8_t btm6)
{
    ASSERT_ERR((((uint32_t)btm6 << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)btm6 << 24));
}

__INLINE uint8_t bt_btmprio0_btm5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void bt_btmprio0_btm5_setf(uint8_t btm5)
{
    ASSERT_ERR((((uint32_t)btm5 << 20) & ~((uint32_t)0x00F00000)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)btm5 << 20));
}

__INLINE uint8_t bt_btmprio0_btm4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_btmprio0_btm4_setf(uint8_t btm4)
{
    ASSERT_ERR((((uint32_t)btm4 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)btm4 << 16));
}

__INLINE uint8_t bt_btmprio0_btm3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void bt_btmprio0_btm3_setf(uint8_t btm3)
{
    ASSERT_ERR((((uint32_t)btm3 << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)btm3 << 12));
}

__INLINE uint8_t bt_btmprio0_btm2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_btmprio0_btm2_setf(uint8_t btm2)
{
    ASSERT_ERR((((uint32_t)btm2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)btm2 << 8));
}

__INLINE uint8_t bt_btmprio0_btm1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bt_btmprio0_btm1_setf(uint8_t btm1)
{
    ASSERT_ERR((((uint32_t)btm1 << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)btm1 << 4));
}

__INLINE uint8_t bt_btmprio0_btm0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_btmprio0_btm0_setf(uint8_t btm0)
{
    ASSERT_ERR((((uint32_t)btm0 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_BTMPRIO0_ADDR, (REG_BT_RD(BT_BTMPRIO0_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)btm0 << 0));
}

/**
 * @brief BTMPRIO1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28                BTM15   0x8
 *  27:24                BTM14   0xF
 *  23:20                BTM13   0x9
 *  19:16                BTM12   0x9
 *  15:12                BTM11   0x9
 *  11:08                BTM10   0x9
 *  07:04                 BTM9   0xA
 *  03:00                 BTM8   0xB
 * </pre>
 */
#define BT_BTMPRIO1_ADDR   0x50090564
#define BT_BTMPRIO1_OFFSET 0x00000164
#define BT_BTMPRIO1_INDEX  0x00000059
#define BT_BTMPRIO1_RESET  0x8F9999AB

__INLINE uint32_t bt_btmprio1_get(void)
{
    return REG_BT_RD(BT_BTMPRIO1_ADDR);
}

__INLINE void bt_btmprio1_set(uint32_t value)
{
    REG_BT_WR(BT_BTMPRIO1_ADDR, value);
}

// field definitions
#define BT_BTM15_MASK   ((uint32_t)0xF0000000)
#define BT_BTM15_LSB    28
#define BT_BTM15_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM14_MASK   ((uint32_t)0x0F000000)
#define BT_BTM14_LSB    24
#define BT_BTM14_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM13_MASK   ((uint32_t)0x00F00000)
#define BT_BTM13_LSB    20
#define BT_BTM13_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM12_MASK   ((uint32_t)0x000F0000)
#define BT_BTM12_LSB    16
#define BT_BTM12_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM11_MASK   ((uint32_t)0x0000F000)
#define BT_BTM11_LSB    12
#define BT_BTM11_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM10_MASK   ((uint32_t)0x00000F00)
#define BT_BTM10_LSB    8
#define BT_BTM10_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM9_MASK    ((uint32_t)0x000000F0)
#define BT_BTM9_LSB     4
#define BT_BTM9_WIDTH   ((uint32_t)0x00000004)
#define BT_BTM8_MASK    ((uint32_t)0x0000000F)
#define BT_BTM8_LSB     0
#define BT_BTM8_WIDTH   ((uint32_t)0x00000004)

#define BT_BTM15_RST    0x8
#define BT_BTM14_RST    0xF
#define BT_BTM13_RST    0x9
#define BT_BTM12_RST    0x9
#define BT_BTM11_RST    0x9
#define BT_BTM10_RST    0x9
#define BT_BTM9_RST     0xA
#define BT_BTM8_RST     0xB

__INLINE void bt_btmprio1_pack(uint8_t btm15, uint8_t btm14, uint8_t btm13, uint8_t btm12, uint8_t btm11, uint8_t btm10, uint8_t btm9, uint8_t btm8)
{
    ASSERT_ERR((((uint32_t)btm15 << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)btm14 << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)btm13 << 20) & ~((uint32_t)0x00F00000)) == 0);
    ASSERT_ERR((((uint32_t)btm12 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)btm11 << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)btm10 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)btm9 << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)btm8 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, ((uint32_t)btm15 << 28) | ((uint32_t)btm14 << 24) | ((uint32_t)btm13 << 20) | ((uint32_t)btm12 << 16) | ((uint32_t)btm11 << 12) | ((uint32_t)btm10 << 8) | ((uint32_t)btm9 << 4) | ((uint32_t)btm8 << 0));
}

__INLINE void bt_btmprio1_unpack(uint8_t *btm15, uint8_t *btm14, uint8_t *btm13, uint8_t *btm12, uint8_t *btm11, uint8_t *btm10, uint8_t *btm9, uint8_t *btm8)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);

    *btm15 = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *btm14 = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *btm13 = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *btm12 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *btm11 = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *btm10 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *btm9 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *btm8 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_btmprio1_btm15_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void bt_btmprio1_btm15_setf(uint8_t btm15)
{
    ASSERT_ERR((((uint32_t)btm15 << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)btm15 << 28));
}

__INLINE uint8_t bt_btmprio1_btm14_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void bt_btmprio1_btm14_setf(uint8_t btm14)
{
    ASSERT_ERR((((uint32_t)btm14 << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)btm14 << 24));
}

__INLINE uint8_t bt_btmprio1_btm13_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void bt_btmprio1_btm13_setf(uint8_t btm13)
{
    ASSERT_ERR((((uint32_t)btm13 << 20) & ~((uint32_t)0x00F00000)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)btm13 << 20));
}

__INLINE uint8_t bt_btmprio1_btm12_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_btmprio1_btm12_setf(uint8_t btm12)
{
    ASSERT_ERR((((uint32_t)btm12 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)btm12 << 16));
}

__INLINE uint8_t bt_btmprio1_btm11_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void bt_btmprio1_btm11_setf(uint8_t btm11)
{
    ASSERT_ERR((((uint32_t)btm11 << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)btm11 << 12));
}

__INLINE uint8_t bt_btmprio1_btm10_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_btmprio1_btm10_setf(uint8_t btm10)
{
    ASSERT_ERR((((uint32_t)btm10 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)btm10 << 8));
}

__INLINE uint8_t bt_btmprio1_btm9_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bt_btmprio1_btm9_setf(uint8_t btm9)
{
    ASSERT_ERR((((uint32_t)btm9 << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)btm9 << 4));
}

__INLINE uint8_t bt_btmprio1_btm8_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_btmprio1_btm8_setf(uint8_t btm8)
{
    ASSERT_ERR((((uint32_t)btm8 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_BTMPRIO1_ADDR, (REG_BT_RD(BT_BTMPRIO1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)btm8 << 0));
}

/**
 * @brief BTMPRIO2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28           BTMDEFAULT   0x3
 *  19:16                BTM20   0x3
 *  15:12                BTM19   0x4
 *  11:08                BTM18   0x5
 *  07:04                BTM17   0x6
 *  03:00                BTM16   0x7
 * </pre>
 */
#define BT_BTMPRIO2_ADDR   0x50090568
#define BT_BTMPRIO2_OFFSET 0x00000168
#define BT_BTMPRIO2_INDEX  0x0000005A
#define BT_BTMPRIO2_RESET  0x30034567

__INLINE uint32_t bt_btmprio2_get(void)
{
    return REG_BT_RD(BT_BTMPRIO2_ADDR);
}

__INLINE void bt_btmprio2_set(uint32_t value)
{
    REG_BT_WR(BT_BTMPRIO2_ADDR, value);
}

// field definitions
#define BT_BTMDEFAULT_MASK   ((uint32_t)0xF0000000)
#define BT_BTMDEFAULT_LSB    28
#define BT_BTMDEFAULT_WIDTH  ((uint32_t)0x00000004)
#define BT_BTM20_MASK        ((uint32_t)0x000F0000)
#define BT_BTM20_LSB         16
#define BT_BTM20_WIDTH       ((uint32_t)0x00000004)
#define BT_BTM19_MASK        ((uint32_t)0x0000F000)
#define BT_BTM19_LSB         12
#define BT_BTM19_WIDTH       ((uint32_t)0x00000004)
#define BT_BTM18_MASK        ((uint32_t)0x00000F00)
#define BT_BTM18_LSB         8
#define BT_BTM18_WIDTH       ((uint32_t)0x00000004)
#define BT_BTM17_MASK        ((uint32_t)0x000000F0)
#define BT_BTM17_LSB         4
#define BT_BTM17_WIDTH       ((uint32_t)0x00000004)
#define BT_BTM16_MASK        ((uint32_t)0x0000000F)
#define BT_BTM16_LSB         0
#define BT_BTM16_WIDTH       ((uint32_t)0x00000004)

#define BT_BTMDEFAULT_RST    0x3
#define BT_BTM20_RST         0x3
#define BT_BTM19_RST         0x4
#define BT_BTM18_RST         0x5
#define BT_BTM17_RST         0x6
#define BT_BTM16_RST         0x7

__INLINE void bt_btmprio2_pack(uint8_t btmdefault, uint8_t btm20, uint8_t btm19, uint8_t btm18, uint8_t btm17, uint8_t btm16)
{
    ASSERT_ERR((((uint32_t)btmdefault << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)btm20 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)btm19 << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)btm18 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)btm17 << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)btm16 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_BTMPRIO2_ADDR, ((uint32_t)btmdefault << 28) | ((uint32_t)btm20 << 16) | ((uint32_t)btm19 << 12) | ((uint32_t)btm18 << 8) | ((uint32_t)btm17 << 4) | ((uint32_t)btm16 << 0));
}

__INLINE void bt_btmprio2_unpack(uint8_t *btmdefault, uint8_t *btm20, uint8_t *btm19, uint8_t *btm18, uint8_t *btm17, uint8_t *btm16)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO2_ADDR);

    *btmdefault = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *btm20 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *btm19 = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *btm18 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *btm17 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *btm16 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_btmprio2_btmdefault_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO2_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void bt_btmprio2_btmdefault_setf(uint8_t btmdefault)
{
    ASSERT_ERR((((uint32_t)btmdefault << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_BT_WR(BT_BTMPRIO2_ADDR, (REG_BT_RD(BT_BTMPRIO2_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)btmdefault << 28));
}

__INLINE uint8_t bt_btmprio2_btm20_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO2_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_btmprio2_btm20_setf(uint8_t btm20)
{
    ASSERT_ERR((((uint32_t)btm20 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_BTMPRIO2_ADDR, (REG_BT_RD(BT_BTMPRIO2_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)btm20 << 16));
}

__INLINE uint8_t bt_btmprio2_btm19_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO2_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void bt_btmprio2_btm19_setf(uint8_t btm19)
{
    ASSERT_ERR((((uint32_t)btm19 << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_BT_WR(BT_BTMPRIO2_ADDR, (REG_BT_RD(BT_BTMPRIO2_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)btm19 << 12));
}

__INLINE uint8_t bt_btmprio2_btm18_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO2_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_btmprio2_btm18_setf(uint8_t btm18)
{
    ASSERT_ERR((((uint32_t)btm18 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_BTMPRIO2_ADDR, (REG_BT_RD(BT_BTMPRIO2_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)btm18 << 8));
}

__INLINE uint8_t bt_btmprio2_btm17_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO2_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bt_btmprio2_btm17_setf(uint8_t btm17)
{
    ASSERT_ERR((((uint32_t)btm17 << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_BT_WR(BT_BTMPRIO2_ADDR, (REG_BT_RD(BT_BTMPRIO2_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)btm17 << 4));
}

__INLINE uint8_t bt_btmprio2_btm16_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_BTMPRIO2_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_btmprio2_btm16_setf(uint8_t btm16)
{
    ASSERT_ERR((((uint32_t)btm16 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_BTMPRIO2_ADDR, (REG_BT_RD(BT_BTMPRIO2_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)btm16 << 0));
}

/**
 * @brief MWSPTABLE0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:20            MWSPAT0_5   0x0
 *  18:16            MWSPAT0_4   0x0
 *  14:12            MWSPAT0_3   0x0
 *  10:08            MWSPAT0_2   0x0
 *  06:04            MWSPAT0_1   0x0
 *  02:00            MWSPAT0_0   0x0
 * </pre>
 */
#define BT_MWSPTABLE0_ADDR   0x50090570
#define BT_MWSPTABLE0_OFFSET 0x00000170
#define BT_MWSPTABLE0_INDEX  0x0000005C
#define BT_MWSPTABLE0_RESET  0x00000000

__INLINE uint32_t bt_mwsptable0_get(void)
{
    return REG_BT_RD(BT_MWSPTABLE0_ADDR);
}

__INLINE void bt_mwsptable0_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTABLE0_ADDR, value);
}

// field definitions
#define BT_MWSPAT0_5_MASK   ((uint32_t)0x00700000)
#define BT_MWSPAT0_5_LSB    20
#define BT_MWSPAT0_5_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT0_4_MASK   ((uint32_t)0x00070000)
#define BT_MWSPAT0_4_LSB    16
#define BT_MWSPAT0_4_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT0_3_MASK   ((uint32_t)0x00007000)
#define BT_MWSPAT0_3_LSB    12
#define BT_MWSPAT0_3_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT0_2_MASK   ((uint32_t)0x00000700)
#define BT_MWSPAT0_2_LSB    8
#define BT_MWSPAT0_2_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT0_1_MASK   ((uint32_t)0x00000070)
#define BT_MWSPAT0_1_LSB    4
#define BT_MWSPAT0_1_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT0_0_MASK   ((uint32_t)0x00000007)
#define BT_MWSPAT0_0_LSB    0
#define BT_MWSPAT0_0_WIDTH  ((uint32_t)0x00000003)

#define BT_MWSPAT0_5_RST    0x0
#define BT_MWSPAT0_4_RST    0x0
#define BT_MWSPAT0_3_RST    0x0
#define BT_MWSPAT0_2_RST    0x0
#define BT_MWSPAT0_1_RST    0x0
#define BT_MWSPAT0_0_RST    0x0

__INLINE void bt_mwsptable0_pack(uint8_t mwspat05, uint8_t mwspat04, uint8_t mwspat03, uint8_t mwspat02, uint8_t mwspat01, uint8_t mwspat00)
{
    ASSERT_ERR((((uint32_t)mwspat05 << 20) & ~((uint32_t)0x00700000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat04 << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat03 << 12) & ~((uint32_t)0x00007000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat02 << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)mwspat01 << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)mwspat00 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSPTABLE0_ADDR, ((uint32_t)mwspat05 << 20) | ((uint32_t)mwspat04 << 16) | ((uint32_t)mwspat03 << 12) | ((uint32_t)mwspat02 << 8) | ((uint32_t)mwspat01 << 4) | ((uint32_t)mwspat00 << 0));
}

__INLINE void bt_mwsptable0_unpack(uint8_t *mwspat05, uint8_t *mwspat04, uint8_t *mwspat03, uint8_t *mwspat02, uint8_t *mwspat01, uint8_t *mwspat00)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE0_ADDR);

    *mwspat05 = (localVal & ((uint32_t)0x00700000)) >> 20;
    *mwspat04 = (localVal & ((uint32_t)0x00070000)) >> 16;
    *mwspat03 = (localVal & ((uint32_t)0x00007000)) >> 12;
    *mwspat02 = (localVal & ((uint32_t)0x00000700)) >> 8;
    *mwspat01 = (localVal & ((uint32_t)0x00000070)) >> 4;
    *mwspat00 = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_mwsptable0_mwspat0_5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x00700000)) >> 20);
}

__INLINE void bt_mwsptable0_mwspat0_5_setf(uint8_t mwspat05)
{
    ASSERT_ERR((((uint32_t)mwspat05 << 20) & ~((uint32_t)0x00700000)) == 0);
    REG_BT_WR(BT_MWSPTABLE0_ADDR, (REG_BT_RD(BT_MWSPTABLE0_ADDR) & ~((uint32_t)0x00700000)) | ((uint32_t)mwspat05 << 20));
}

__INLINE uint8_t bt_mwsptable0_mwspat0_4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void bt_mwsptable0_mwspat0_4_setf(uint8_t mwspat04)
{
    ASSERT_ERR((((uint32_t)mwspat04 << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_BT_WR(BT_MWSPTABLE0_ADDR, (REG_BT_RD(BT_MWSPTABLE0_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)mwspat04 << 16));
}

__INLINE uint8_t bt_mwsptable0_mwspat0_3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void bt_mwsptable0_mwspat0_3_setf(uint8_t mwspat03)
{
    ASSERT_ERR((((uint32_t)mwspat03 << 12) & ~((uint32_t)0x00007000)) == 0);
    REG_BT_WR(BT_MWSPTABLE0_ADDR, (REG_BT_RD(BT_MWSPTABLE0_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)mwspat03 << 12));
}

__INLINE uint8_t bt_mwsptable0_mwspat0_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void bt_mwsptable0_mwspat0_2_setf(uint8_t mwspat02)
{
    ASSERT_ERR((((uint32_t)mwspat02 << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_BT_WR(BT_MWSPTABLE0_ADDR, (REG_BT_RD(BT_MWSPTABLE0_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)mwspat02 << 8));
}

__INLINE uint8_t bt_mwsptable0_mwspat0_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void bt_mwsptable0_mwspat0_1_setf(uint8_t mwspat01)
{
    ASSERT_ERR((((uint32_t)mwspat01 << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_BT_WR(BT_MWSPTABLE0_ADDR, (REG_BT_RD(BT_MWSPTABLE0_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)mwspat01 << 4));
}

__INLINE uint8_t bt_mwsptable0_mwspat0_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_mwsptable0_mwspat0_0_setf(uint8_t mwspat00)
{
    ASSERT_ERR((((uint32_t)mwspat00 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSPTABLE0_ADDR, (REG_BT_RD(BT_MWSPTABLE0_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)mwspat00 << 0));
}

/**
 * @brief MWSPTIMING00 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16      MWSPTIM0_1_TO_2   0x0
 *  15:00      MWSPTIM0_0_TO_1   0x0
 * </pre>
 */
#define BT_MWSPTIMING00_ADDR   0x50090574
#define BT_MWSPTIMING00_OFFSET 0x00000174
#define BT_MWSPTIMING00_INDEX  0x0000005D
#define BT_MWSPTIMING00_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming00_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING00_ADDR);
}

__INLINE void bt_mwsptiming00_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING00_ADDR, value);
}

// field definitions
#define BT_MWSPTIM0_1_TO_2_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSPTIM0_1_TO_2_LSB    16
#define BT_MWSPTIM0_1_TO_2_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSPTIM0_0_TO_1_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM0_0_TO_1_LSB    0
#define BT_MWSPTIM0_0_TO_1_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM0_1_TO_2_RST    0x0
#define BT_MWSPTIM0_0_TO_1_RST    0x0

__INLINE void bt_mwsptiming00_pack(uint16_t mwsptim01to2, uint16_t mwsptim00to1)
{
    ASSERT_ERR((((uint32_t)mwsptim01to2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsptim00to1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING00_ADDR, ((uint32_t)mwsptim01to2 << 16) | ((uint32_t)mwsptim00to1 << 0));
}

__INLINE void bt_mwsptiming00_unpack(uint16_t *mwsptim01to2, uint16_t *mwsptim00to1)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING00_ADDR);

    *mwsptim01to2 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsptim00to1 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwsptiming00_mwsptim0_1_to_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING00_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwsptiming00_mwsptim0_1_to_2_setf(uint16_t mwsptim01to2)
{
    ASSERT_ERR((((uint32_t)mwsptim01to2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSPTIMING00_ADDR, (REG_BT_RD(BT_MWSPTIMING00_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsptim01to2 << 16));
}

__INLINE uint16_t bt_mwsptiming00_mwsptim0_0_to_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING00_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwsptiming00_mwsptim0_0_to_1_setf(uint16_t mwsptim00to1)
{
    ASSERT_ERR((((uint32_t)mwsptim00to1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING00_ADDR, (REG_BT_RD(BT_MWSPTIMING00_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsptim00to1 << 0));
}

/**
 * @brief MWSPTIMING01 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16      MWSPTIM0_3_TO_4   0x0
 *  15:00      MWSPTIM0_2_TO_3   0x0
 * </pre>
 */
#define BT_MWSPTIMING01_ADDR   0x50090578
#define BT_MWSPTIMING01_OFFSET 0x00000178
#define BT_MWSPTIMING01_INDEX  0x0000005E
#define BT_MWSPTIMING01_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming01_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING01_ADDR);
}

__INLINE void bt_mwsptiming01_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING01_ADDR, value);
}

// field definitions
#define BT_MWSPTIM0_3_TO_4_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSPTIM0_3_TO_4_LSB    16
#define BT_MWSPTIM0_3_TO_4_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSPTIM0_2_TO_3_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM0_2_TO_3_LSB    0
#define BT_MWSPTIM0_2_TO_3_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM0_3_TO_4_RST    0x0
#define BT_MWSPTIM0_2_TO_3_RST    0x0

__INLINE void bt_mwsptiming01_pack(uint16_t mwsptim03to4, uint16_t mwsptim02to3)
{
    ASSERT_ERR((((uint32_t)mwsptim03to4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsptim02to3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING01_ADDR, ((uint32_t)mwsptim03to4 << 16) | ((uint32_t)mwsptim02to3 << 0));
}

__INLINE void bt_mwsptiming01_unpack(uint16_t *mwsptim03to4, uint16_t *mwsptim02to3)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING01_ADDR);

    *mwsptim03to4 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsptim02to3 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwsptiming01_mwsptim0_3_to_4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING01_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwsptiming01_mwsptim0_3_to_4_setf(uint16_t mwsptim03to4)
{
    ASSERT_ERR((((uint32_t)mwsptim03to4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSPTIMING01_ADDR, (REG_BT_RD(BT_MWSPTIMING01_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsptim03to4 << 16));
}

__INLINE uint16_t bt_mwsptiming01_mwsptim0_2_to_3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING01_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwsptiming01_mwsptim0_2_to_3_setf(uint16_t mwsptim02to3)
{
    ASSERT_ERR((((uint32_t)mwsptim02to3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING01_ADDR, (REG_BT_RD(BT_MWSPTIMING01_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsptim02to3 << 0));
}

/**
 * @brief MWSPTIMING02 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00      MWSPTIM0_4_TO_5   0x0
 * </pre>
 */
#define BT_MWSPTIMING02_ADDR   0x5009057C
#define BT_MWSPTIMING02_OFFSET 0x0000017C
#define BT_MWSPTIMING02_INDEX  0x0000005F
#define BT_MWSPTIMING02_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming02_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING02_ADDR);
}

__INLINE void bt_mwsptiming02_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING02_ADDR, value);
}

// field definitions
#define BT_MWSPTIM0_4_TO_5_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM0_4_TO_5_LSB    0
#define BT_MWSPTIM0_4_TO_5_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM0_4_TO_5_RST    0x0

__INLINE uint16_t bt_mwsptiming02_mwsptim0_4_to_5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING02_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_mwsptiming02_mwsptim0_4_to_5_setf(uint16_t mwsptim04to5)
{
    ASSERT_ERR((((uint32_t)mwsptim04to5 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING02_ADDR, (uint32_t)mwsptim04to5 << 0);
}

/**
 * @brief MWSPTABLE1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:20            MWSPAT1_5   0x0
 *  18:16            MWSPAT1_4   0x0
 *  14:12            MWSPAT1_3   0x0
 *  10:08            MWSPAT1_2   0x0
 *  06:04            MWSPAT1_1   0x0
 *  02:00            MWSPAT1_0   0x0
 * </pre>
 */
#define BT_MWSPTABLE1_ADDR   0x50090580
#define BT_MWSPTABLE1_OFFSET 0x00000180
#define BT_MWSPTABLE1_INDEX  0x00000060
#define BT_MWSPTABLE1_RESET  0x00000000

__INLINE uint32_t bt_mwsptable1_get(void)
{
    return REG_BT_RD(BT_MWSPTABLE1_ADDR);
}

__INLINE void bt_mwsptable1_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTABLE1_ADDR, value);
}

// field definitions
#define BT_MWSPAT1_5_MASK   ((uint32_t)0x00700000)
#define BT_MWSPAT1_5_LSB    20
#define BT_MWSPAT1_5_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT1_4_MASK   ((uint32_t)0x00070000)
#define BT_MWSPAT1_4_LSB    16
#define BT_MWSPAT1_4_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT1_3_MASK   ((uint32_t)0x00007000)
#define BT_MWSPAT1_3_LSB    12
#define BT_MWSPAT1_3_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT1_2_MASK   ((uint32_t)0x00000700)
#define BT_MWSPAT1_2_LSB    8
#define BT_MWSPAT1_2_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT1_1_MASK   ((uint32_t)0x00000070)
#define BT_MWSPAT1_1_LSB    4
#define BT_MWSPAT1_1_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT1_0_MASK   ((uint32_t)0x00000007)
#define BT_MWSPAT1_0_LSB    0
#define BT_MWSPAT1_0_WIDTH  ((uint32_t)0x00000003)

#define BT_MWSPAT1_5_RST    0x0
#define BT_MWSPAT1_4_RST    0x0
#define BT_MWSPAT1_3_RST    0x0
#define BT_MWSPAT1_2_RST    0x0
#define BT_MWSPAT1_1_RST    0x0
#define BT_MWSPAT1_0_RST    0x0

__INLINE void bt_mwsptable1_pack(uint8_t mwspat15, uint8_t mwspat14, uint8_t mwspat13, uint8_t mwspat12, uint8_t mwspat11, uint8_t mwspat10)
{
    ASSERT_ERR((((uint32_t)mwspat15 << 20) & ~((uint32_t)0x00700000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat14 << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat13 << 12) & ~((uint32_t)0x00007000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat12 << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)mwspat11 << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)mwspat10 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSPTABLE1_ADDR, ((uint32_t)mwspat15 << 20) | ((uint32_t)mwspat14 << 16) | ((uint32_t)mwspat13 << 12) | ((uint32_t)mwspat12 << 8) | ((uint32_t)mwspat11 << 4) | ((uint32_t)mwspat10 << 0));
}

__INLINE void bt_mwsptable1_unpack(uint8_t *mwspat15, uint8_t *mwspat14, uint8_t *mwspat13, uint8_t *mwspat12, uint8_t *mwspat11, uint8_t *mwspat10)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE1_ADDR);

    *mwspat15 = (localVal & ((uint32_t)0x00700000)) >> 20;
    *mwspat14 = (localVal & ((uint32_t)0x00070000)) >> 16;
    *mwspat13 = (localVal & ((uint32_t)0x00007000)) >> 12;
    *mwspat12 = (localVal & ((uint32_t)0x00000700)) >> 8;
    *mwspat11 = (localVal & ((uint32_t)0x00000070)) >> 4;
    *mwspat10 = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_mwsptable1_mwspat1_5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE1_ADDR);
    return ((localVal & ((uint32_t)0x00700000)) >> 20);
}

__INLINE void bt_mwsptable1_mwspat1_5_setf(uint8_t mwspat15)
{
    ASSERT_ERR((((uint32_t)mwspat15 << 20) & ~((uint32_t)0x00700000)) == 0);
    REG_BT_WR(BT_MWSPTABLE1_ADDR, (REG_BT_RD(BT_MWSPTABLE1_ADDR) & ~((uint32_t)0x00700000)) | ((uint32_t)mwspat15 << 20));
}

__INLINE uint8_t bt_mwsptable1_mwspat1_4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE1_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void bt_mwsptable1_mwspat1_4_setf(uint8_t mwspat14)
{
    ASSERT_ERR((((uint32_t)mwspat14 << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_BT_WR(BT_MWSPTABLE1_ADDR, (REG_BT_RD(BT_MWSPTABLE1_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)mwspat14 << 16));
}

__INLINE uint8_t bt_mwsptable1_mwspat1_3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE1_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void bt_mwsptable1_mwspat1_3_setf(uint8_t mwspat13)
{
    ASSERT_ERR((((uint32_t)mwspat13 << 12) & ~((uint32_t)0x00007000)) == 0);
    REG_BT_WR(BT_MWSPTABLE1_ADDR, (REG_BT_RD(BT_MWSPTABLE1_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)mwspat13 << 12));
}

__INLINE uint8_t bt_mwsptable1_mwspat1_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE1_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void bt_mwsptable1_mwspat1_2_setf(uint8_t mwspat12)
{
    ASSERT_ERR((((uint32_t)mwspat12 << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_BT_WR(BT_MWSPTABLE1_ADDR, (REG_BT_RD(BT_MWSPTABLE1_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)mwspat12 << 8));
}

__INLINE uint8_t bt_mwsptable1_mwspat1_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE1_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void bt_mwsptable1_mwspat1_1_setf(uint8_t mwspat11)
{
    ASSERT_ERR((((uint32_t)mwspat11 << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_BT_WR(BT_MWSPTABLE1_ADDR, (REG_BT_RD(BT_MWSPTABLE1_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)mwspat11 << 4));
}

__INLINE uint8_t bt_mwsptable1_mwspat1_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE1_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_mwsptable1_mwspat1_0_setf(uint8_t mwspat10)
{
    ASSERT_ERR((((uint32_t)mwspat10 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSPTABLE1_ADDR, (REG_BT_RD(BT_MWSPTABLE1_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)mwspat10 << 0));
}

/**
 * @brief MWSPTIMING10 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16      MWSPTIM1_1_TO_2   0x0
 *  15:00      MWSPTIM1_0_TO_1   0x0
 * </pre>
 */
#define BT_MWSPTIMING10_ADDR   0x50090584
#define BT_MWSPTIMING10_OFFSET 0x00000184
#define BT_MWSPTIMING10_INDEX  0x00000061
#define BT_MWSPTIMING10_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming10_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING10_ADDR);
}

__INLINE void bt_mwsptiming10_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING10_ADDR, value);
}

// field definitions
#define BT_MWSPTIM1_1_TO_2_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSPTIM1_1_TO_2_LSB    16
#define BT_MWSPTIM1_1_TO_2_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSPTIM1_0_TO_1_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM1_0_TO_1_LSB    0
#define BT_MWSPTIM1_0_TO_1_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM1_1_TO_2_RST    0x0
#define BT_MWSPTIM1_0_TO_1_RST    0x0

__INLINE void bt_mwsptiming10_pack(uint16_t mwsptim11to2, uint16_t mwsptim10to1)
{
    ASSERT_ERR((((uint32_t)mwsptim11to2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsptim10to1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING10_ADDR, ((uint32_t)mwsptim11to2 << 16) | ((uint32_t)mwsptim10to1 << 0));
}

__INLINE void bt_mwsptiming10_unpack(uint16_t *mwsptim11to2, uint16_t *mwsptim10to1)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING10_ADDR);

    *mwsptim11to2 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsptim10to1 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwsptiming10_mwsptim1_1_to_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING10_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwsptiming10_mwsptim1_1_to_2_setf(uint16_t mwsptim11to2)
{
    ASSERT_ERR((((uint32_t)mwsptim11to2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSPTIMING10_ADDR, (REG_BT_RD(BT_MWSPTIMING10_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsptim11to2 << 16));
}

__INLINE uint16_t bt_mwsptiming10_mwsptim1_0_to_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING10_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwsptiming10_mwsptim1_0_to_1_setf(uint16_t mwsptim10to1)
{
    ASSERT_ERR((((uint32_t)mwsptim10to1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING10_ADDR, (REG_BT_RD(BT_MWSPTIMING10_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsptim10to1 << 0));
}

/**
 * @brief MWSPTIMING11 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16      MWSPTIM1_3_TO_4   0x0
 *  15:00      MWSPTIM1_2_TO_3   0x0
 * </pre>
 */
#define BT_MWSPTIMING11_ADDR   0x50090588
#define BT_MWSPTIMING11_OFFSET 0x00000188
#define BT_MWSPTIMING11_INDEX  0x00000062
#define BT_MWSPTIMING11_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming11_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING11_ADDR);
}

__INLINE void bt_mwsptiming11_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING11_ADDR, value);
}

// field definitions
#define BT_MWSPTIM1_3_TO_4_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSPTIM1_3_TO_4_LSB    16
#define BT_MWSPTIM1_3_TO_4_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSPTIM1_2_TO_3_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM1_2_TO_3_LSB    0
#define BT_MWSPTIM1_2_TO_3_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM1_3_TO_4_RST    0x0
#define BT_MWSPTIM1_2_TO_3_RST    0x0

__INLINE void bt_mwsptiming11_pack(uint16_t mwsptim13to4, uint16_t mwsptim12to3)
{
    ASSERT_ERR((((uint32_t)mwsptim13to4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsptim12to3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING11_ADDR, ((uint32_t)mwsptim13to4 << 16) | ((uint32_t)mwsptim12to3 << 0));
}

__INLINE void bt_mwsptiming11_unpack(uint16_t *mwsptim13to4, uint16_t *mwsptim12to3)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING11_ADDR);

    *mwsptim13to4 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsptim12to3 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwsptiming11_mwsptim1_3_to_4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING11_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwsptiming11_mwsptim1_3_to_4_setf(uint16_t mwsptim13to4)
{
    ASSERT_ERR((((uint32_t)mwsptim13to4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSPTIMING11_ADDR, (REG_BT_RD(BT_MWSPTIMING11_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsptim13to4 << 16));
}

__INLINE uint16_t bt_mwsptiming11_mwsptim1_2_to_3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING11_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwsptiming11_mwsptim1_2_to_3_setf(uint16_t mwsptim12to3)
{
    ASSERT_ERR((((uint32_t)mwsptim12to3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING11_ADDR, (REG_BT_RD(BT_MWSPTIMING11_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsptim12to3 << 0));
}

/**
 * @brief MWSPTIMING12 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00      MWSPTIM1_4_TO_5   0x0
 * </pre>
 */
#define BT_MWSPTIMING12_ADDR   0x5009058C
#define BT_MWSPTIMING12_OFFSET 0x0000018C
#define BT_MWSPTIMING12_INDEX  0x00000063
#define BT_MWSPTIMING12_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming12_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING12_ADDR);
}

__INLINE void bt_mwsptiming12_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING12_ADDR, value);
}

// field definitions
#define BT_MWSPTIM1_4_TO_5_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM1_4_TO_5_LSB    0
#define BT_MWSPTIM1_4_TO_5_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM1_4_TO_5_RST    0x0

__INLINE uint16_t bt_mwsptiming12_mwsptim1_4_to_5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING12_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_mwsptiming12_mwsptim1_4_to_5_setf(uint16_t mwsptim14to5)
{
    ASSERT_ERR((((uint32_t)mwsptim14to5 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING12_ADDR, (uint32_t)mwsptim14to5 << 0);
}

/**
 * @brief MWSPTABLE2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:20            MWSPAT2_5   0x0
 *  18:16            MWSPAT2_4   0x0
 *  14:12            MWSPAT2_3   0x0
 *  10:08            MWSPAT2_2   0x0
 *  06:04            MWSPAT2_1   0x0
 *  02:00            MWSPAT2_0   0x0
 * </pre>
 */
#define BT_MWSPTABLE2_ADDR   0x50090590
#define BT_MWSPTABLE2_OFFSET 0x00000190
#define BT_MWSPTABLE2_INDEX  0x00000064
#define BT_MWSPTABLE2_RESET  0x00000000

__INLINE uint32_t bt_mwsptable2_get(void)
{
    return REG_BT_RD(BT_MWSPTABLE2_ADDR);
}

__INLINE void bt_mwsptable2_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTABLE2_ADDR, value);
}

// field definitions
#define BT_MWSPAT2_5_MASK   ((uint32_t)0x00700000)
#define BT_MWSPAT2_5_LSB    20
#define BT_MWSPAT2_5_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT2_4_MASK   ((uint32_t)0x00070000)
#define BT_MWSPAT2_4_LSB    16
#define BT_MWSPAT2_4_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT2_3_MASK   ((uint32_t)0x00007000)
#define BT_MWSPAT2_3_LSB    12
#define BT_MWSPAT2_3_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT2_2_MASK   ((uint32_t)0x00000700)
#define BT_MWSPAT2_2_LSB    8
#define BT_MWSPAT2_2_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT2_1_MASK   ((uint32_t)0x00000070)
#define BT_MWSPAT2_1_LSB    4
#define BT_MWSPAT2_1_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSPAT2_0_MASK   ((uint32_t)0x00000007)
#define BT_MWSPAT2_0_LSB    0
#define BT_MWSPAT2_0_WIDTH  ((uint32_t)0x00000003)

#define BT_MWSPAT2_5_RST    0x0
#define BT_MWSPAT2_4_RST    0x0
#define BT_MWSPAT2_3_RST    0x0
#define BT_MWSPAT2_2_RST    0x0
#define BT_MWSPAT2_1_RST    0x0
#define BT_MWSPAT2_0_RST    0x0

__INLINE void bt_mwsptable2_pack(uint8_t mwspat25, uint8_t mwspat24, uint8_t mwspat23, uint8_t mwspat22, uint8_t mwspat21, uint8_t mwspat20)
{
    ASSERT_ERR((((uint32_t)mwspat25 << 20) & ~((uint32_t)0x00700000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat24 << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat23 << 12) & ~((uint32_t)0x00007000)) == 0);
    ASSERT_ERR((((uint32_t)mwspat22 << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)mwspat21 << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)mwspat20 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSPTABLE2_ADDR, ((uint32_t)mwspat25 << 20) | ((uint32_t)mwspat24 << 16) | ((uint32_t)mwspat23 << 12) | ((uint32_t)mwspat22 << 8) | ((uint32_t)mwspat21 << 4) | ((uint32_t)mwspat20 << 0));
}

__INLINE void bt_mwsptable2_unpack(uint8_t *mwspat25, uint8_t *mwspat24, uint8_t *mwspat23, uint8_t *mwspat22, uint8_t *mwspat21, uint8_t *mwspat20)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE2_ADDR);

    *mwspat25 = (localVal & ((uint32_t)0x00700000)) >> 20;
    *mwspat24 = (localVal & ((uint32_t)0x00070000)) >> 16;
    *mwspat23 = (localVal & ((uint32_t)0x00007000)) >> 12;
    *mwspat22 = (localVal & ((uint32_t)0x00000700)) >> 8;
    *mwspat21 = (localVal & ((uint32_t)0x00000070)) >> 4;
    *mwspat20 = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_mwsptable2_mwspat2_5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE2_ADDR);
    return ((localVal & ((uint32_t)0x00700000)) >> 20);
}

__INLINE void bt_mwsptable2_mwspat2_5_setf(uint8_t mwspat25)
{
    ASSERT_ERR((((uint32_t)mwspat25 << 20) & ~((uint32_t)0x00700000)) == 0);
    REG_BT_WR(BT_MWSPTABLE2_ADDR, (REG_BT_RD(BT_MWSPTABLE2_ADDR) & ~((uint32_t)0x00700000)) | ((uint32_t)mwspat25 << 20));
}

__INLINE uint8_t bt_mwsptable2_mwspat2_4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE2_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void bt_mwsptable2_mwspat2_4_setf(uint8_t mwspat24)
{
    ASSERT_ERR((((uint32_t)mwspat24 << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_BT_WR(BT_MWSPTABLE2_ADDR, (REG_BT_RD(BT_MWSPTABLE2_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)mwspat24 << 16));
}

__INLINE uint8_t bt_mwsptable2_mwspat2_3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE2_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void bt_mwsptable2_mwspat2_3_setf(uint8_t mwspat23)
{
    ASSERT_ERR((((uint32_t)mwspat23 << 12) & ~((uint32_t)0x00007000)) == 0);
    REG_BT_WR(BT_MWSPTABLE2_ADDR, (REG_BT_RD(BT_MWSPTABLE2_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)mwspat23 << 12));
}

__INLINE uint8_t bt_mwsptable2_mwspat2_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE2_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void bt_mwsptable2_mwspat2_2_setf(uint8_t mwspat22)
{
    ASSERT_ERR((((uint32_t)mwspat22 << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_BT_WR(BT_MWSPTABLE2_ADDR, (REG_BT_RD(BT_MWSPTABLE2_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)mwspat22 << 8));
}

__INLINE uint8_t bt_mwsptable2_mwspat2_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE2_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void bt_mwsptable2_mwspat2_1_setf(uint8_t mwspat21)
{
    ASSERT_ERR((((uint32_t)mwspat21 << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_BT_WR(BT_MWSPTABLE2_ADDR, (REG_BT_RD(BT_MWSPTABLE2_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)mwspat21 << 4));
}

__INLINE uint8_t bt_mwsptable2_mwspat2_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTABLE2_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_mwsptable2_mwspat2_0_setf(uint8_t mwspat20)
{
    ASSERT_ERR((((uint32_t)mwspat20 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSPTABLE2_ADDR, (REG_BT_RD(BT_MWSPTABLE2_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)mwspat20 << 0));
}

/**
 * @brief MWSPTIMING20 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16      MWSPTIM2_1_TO_2   0x0
 *  15:00      MWSPTIM2_0_TO_1   0x0
 * </pre>
 */
#define BT_MWSPTIMING20_ADDR   0x50090594
#define BT_MWSPTIMING20_OFFSET 0x00000194
#define BT_MWSPTIMING20_INDEX  0x00000065
#define BT_MWSPTIMING20_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming20_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING20_ADDR);
}

__INLINE void bt_mwsptiming20_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING20_ADDR, value);
}

// field definitions
#define BT_MWSPTIM2_1_TO_2_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSPTIM2_1_TO_2_LSB    16
#define BT_MWSPTIM2_1_TO_2_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSPTIM2_0_TO_1_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM2_0_TO_1_LSB    0
#define BT_MWSPTIM2_0_TO_1_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM2_1_TO_2_RST    0x0
#define BT_MWSPTIM2_0_TO_1_RST    0x0

__INLINE void bt_mwsptiming20_pack(uint16_t mwsptim21to2, uint16_t mwsptim20to1)
{
    ASSERT_ERR((((uint32_t)mwsptim21to2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsptim20to1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING20_ADDR, ((uint32_t)mwsptim21to2 << 16) | ((uint32_t)mwsptim20to1 << 0));
}

__INLINE void bt_mwsptiming20_unpack(uint16_t *mwsptim21to2, uint16_t *mwsptim20to1)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING20_ADDR);

    *mwsptim21to2 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsptim20to1 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwsptiming20_mwsptim2_1_to_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING20_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwsptiming20_mwsptim2_1_to_2_setf(uint16_t mwsptim21to2)
{
    ASSERT_ERR((((uint32_t)mwsptim21to2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSPTIMING20_ADDR, (REG_BT_RD(BT_MWSPTIMING20_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsptim21to2 << 16));
}

__INLINE uint16_t bt_mwsptiming20_mwsptim2_0_to_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING20_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwsptiming20_mwsptim2_0_to_1_setf(uint16_t mwsptim20to1)
{
    ASSERT_ERR((((uint32_t)mwsptim20to1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING20_ADDR, (REG_BT_RD(BT_MWSPTIMING20_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsptim20to1 << 0));
}

/**
 * @brief MWSPTIMING21 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16      MWSPTIM2_3_TO_4   0x0
 *  15:00      MWSPTIM2_2_TO_3   0x0
 * </pre>
 */
#define BT_MWSPTIMING21_ADDR   0x50090598
#define BT_MWSPTIMING21_OFFSET 0x00000198
#define BT_MWSPTIMING21_INDEX  0x00000066
#define BT_MWSPTIMING21_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming21_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING21_ADDR);
}

__INLINE void bt_mwsptiming21_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING21_ADDR, value);
}

// field definitions
#define BT_MWSPTIM2_3_TO_4_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSPTIM2_3_TO_4_LSB    16
#define BT_MWSPTIM2_3_TO_4_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSPTIM2_2_TO_3_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM2_2_TO_3_LSB    0
#define BT_MWSPTIM2_2_TO_3_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM2_3_TO_4_RST    0x0
#define BT_MWSPTIM2_2_TO_3_RST    0x0

__INLINE void bt_mwsptiming21_pack(uint16_t mwsptim23to4, uint16_t mwsptim22to3)
{
    ASSERT_ERR((((uint32_t)mwsptim23to4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsptim22to3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING21_ADDR, ((uint32_t)mwsptim23to4 << 16) | ((uint32_t)mwsptim22to3 << 0));
}

__INLINE void bt_mwsptiming21_unpack(uint16_t *mwsptim23to4, uint16_t *mwsptim22to3)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING21_ADDR);

    *mwsptim23to4 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsptim22to3 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwsptiming21_mwsptim2_3_to_4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING21_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwsptiming21_mwsptim2_3_to_4_setf(uint16_t mwsptim23to4)
{
    ASSERT_ERR((((uint32_t)mwsptim23to4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSPTIMING21_ADDR, (REG_BT_RD(BT_MWSPTIMING21_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsptim23to4 << 16));
}

__INLINE uint16_t bt_mwsptiming21_mwsptim2_2_to_3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING21_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwsptiming21_mwsptim2_2_to_3_setf(uint16_t mwsptim22to3)
{
    ASSERT_ERR((((uint32_t)mwsptim22to3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING21_ADDR, (REG_BT_RD(BT_MWSPTIMING21_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsptim22to3 << 0));
}

/**
 * @brief MWSPTIMING22 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00      MWSPTIM2_4_TO_5   0x0
 * </pre>
 */
#define BT_MWSPTIMING22_ADDR   0x5009059C
#define BT_MWSPTIMING22_OFFSET 0x0000019C
#define BT_MWSPTIMING22_INDEX  0x00000067
#define BT_MWSPTIMING22_RESET  0x00000000

__INLINE uint32_t bt_mwsptiming22_get(void)
{
    return REG_BT_RD(BT_MWSPTIMING22_ADDR);
}

__INLINE void bt_mwsptiming22_set(uint32_t value)
{
    REG_BT_WR(BT_MWSPTIMING22_ADDR, value);
}

// field definitions
#define BT_MWSPTIM2_4_TO_5_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSPTIM2_4_TO_5_LSB    0
#define BT_MWSPTIM2_4_TO_5_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSPTIM2_4_TO_5_RST    0x0

__INLINE uint16_t bt_mwsptiming22_mwsptim2_4_to_5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSPTIMING22_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_mwsptiming22_mwsptim2_4_to_5_setf(uint16_t mwsptim24to5)
{
    ASSERT_ERR((((uint32_t)mwsptim24to5 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSPTIMING22_ADDR, (uint32_t)mwsptim24to5 << 0);
}

/**
 * @brief MWSIFSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31          MWS_IF_SAMP   0
 *  08:04   MWS_INACTIVITY_DURATION_VAL   0x0
 *     02   MWS_PATTERN_START_IND_VAL   0
 *  01:00      MWS_PATTERN_VAL   0x0
 * </pre>
 */
#define BT_MWSIFSTAT_ADDR   0x500905A0
#define BT_MWSIFSTAT_OFFSET 0x000001A0
#define BT_MWSIFSTAT_INDEX  0x00000068
#define BT_MWSIFSTAT_RESET  0x00000000

__INLINE uint32_t bt_mwsifstat_get(void)
{
    return REG_BT_RD(BT_MWSIFSTAT_ADDR);
}

__INLINE void bt_mwsifstat_set(uint32_t value)
{
    REG_BT_WR(BT_MWSIFSTAT_ADDR, value);
}

// field definitions
#define BT_MWS_IF_SAMP_BIT                    ((uint32_t)0x80000000)
#define BT_MWS_IF_SAMP_POS                    31
#define BT_MWS_INACTIVITY_DURATION_VAL_MASK   ((uint32_t)0x000001F0)
#define BT_MWS_INACTIVITY_DURATION_VAL_LSB    4
#define BT_MWS_INACTIVITY_DURATION_VAL_WIDTH  ((uint32_t)0x00000005)
#define BT_MWS_PATTERN_START_IND_VAL_BIT      ((uint32_t)0x00000004)
#define BT_MWS_PATTERN_START_IND_VAL_POS      2
#define BT_MWS_PATTERN_VAL_MASK               ((uint32_t)0x00000003)
#define BT_MWS_PATTERN_VAL_LSB                0
#define BT_MWS_PATTERN_VAL_WIDTH              ((uint32_t)0x00000002)

#define BT_MWS_IF_SAMP_RST                    0x0
#define BT_MWS_INACTIVITY_DURATION_VAL_RST    0x0
#define BT_MWS_PATTERN_START_IND_VAL_RST      0x0
#define BT_MWS_PATTERN_VAL_RST                0x0

__INLINE void bt_mwsifstat_unpack(uint8_t *mwsifsamp, uint8_t *mwsinactivitydurationval, uint8_t *mwspatternstartindval, uint8_t *mwspatternval)
{
    uint32_t localVal = REG_BT_RD(BT_MWSIFSTAT_ADDR);

    *mwsifsamp = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mwsinactivitydurationval = (localVal & ((uint32_t)0x000001F0)) >> 4;
    *mwspatternstartindval = (localVal & ((uint32_t)0x00000004)) >> 2;
    *mwspatternval = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t bt_mwsifstat_mws_if_samp_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSIFSTAT_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_mwsifstat_mws_if_samp_setf(uint8_t mwsifsamp)
{
    ASSERT_ERR((((uint32_t)mwsifsamp << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_MWSIFSTAT_ADDR, (uint32_t)mwsifsamp << 31);
}

__INLINE uint8_t bt_mwsifstat_mws_inactivity_duration_val_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSIFSTAT_ADDR);
    return ((localVal & ((uint32_t)0x000001F0)) >> 4);
}

__INLINE uint8_t bt_mwsifstat_mws_pattern_start_ind_val_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSIFSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t bt_mwsifstat_mws_pattern_val_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSIFSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

/**
 * @brief MWSTXTABLE0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           MWSTXFREQH   0x0
 *  15:00           MWSTXFREQL   0xFFFF
 * </pre>
 */
#define BT_MWSTXTABLE0_ADDR   0x500905B0
#define BT_MWSTXTABLE0_OFFSET 0x000001B0
#define BT_MWSTXTABLE0_INDEX  0x0000006C
#define BT_MWSTXTABLE0_RESET  0x0000FFFF

__INLINE uint32_t bt_mwstxtable0_get(void)
{
    return REG_BT_RD(BT_MWSTXTABLE0_ADDR);
}

__INLINE void bt_mwstxtable0_set(uint32_t value)
{
    REG_BT_WR(BT_MWSTXTABLE0_ADDR, value);
}

// field definitions
#define BT_MWSTXFREQH_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSTXFREQH_LSB    16
#define BT_MWSTXFREQH_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSTXFREQL_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSTXFREQL_LSB    0
#define BT_MWSTXFREQL_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSTXFREQH_RST    0x0
#define BT_MWSTXFREQL_RST    0xFFFF

__INLINE void bt_mwstxtable0_pack(uint16_t mwstxfreqh, uint16_t mwstxfreql)
{
    ASSERT_ERR((((uint32_t)mwstxfreqh << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwstxfreql << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSTXTABLE0_ADDR, ((uint32_t)mwstxfreqh << 16) | ((uint32_t)mwstxfreql << 0));
}

__INLINE void bt_mwstxtable0_unpack(uint16_t *mwstxfreqh, uint16_t *mwstxfreql)
{
    uint32_t localVal = REG_BT_RD(BT_MWSTXTABLE0_ADDR);

    *mwstxfreqh = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwstxfreql = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwstxtable0_mwstxfreqh_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSTXTABLE0_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwstxtable0_mwstxfreqh_setf(uint16_t mwstxfreqh)
{
    ASSERT_ERR((((uint32_t)mwstxfreqh << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSTXTABLE0_ADDR, (REG_BT_RD(BT_MWSTXTABLE0_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwstxfreqh << 16));
}

__INLINE uint16_t bt_mwstxtable0_mwstxfreql_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSTXTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwstxtable0_mwstxfreql_setf(uint16_t mwstxfreql)
{
    ASSERT_ERR((((uint32_t)mwstxfreql << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSTXTABLE0_ADDR, (REG_BT_RD(BT_MWSTXTABLE0_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwstxfreql << 0));
}

/**
 * @brief MWSRXTABLE0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           MWSRXFREQH   0x0
 *  15:00           MWSRXFREQL   0xFFFF
 * </pre>
 */
#define BT_MWSRXTABLE0_ADDR   0x500905B4
#define BT_MWSRXTABLE0_OFFSET 0x000001B4
#define BT_MWSRXTABLE0_INDEX  0x0000006D
#define BT_MWSRXTABLE0_RESET  0x0000FFFF

__INLINE uint32_t bt_mwsrxtable0_get(void)
{
    return REG_BT_RD(BT_MWSRXTABLE0_ADDR);
}

__INLINE void bt_mwsrxtable0_set(uint32_t value)
{
    REG_BT_WR(BT_MWSRXTABLE0_ADDR, value);
}

// field definitions
#define BT_MWSRXFREQH_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSRXFREQH_LSB    16
#define BT_MWSRXFREQH_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSRXFREQL_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSRXFREQL_LSB    0
#define BT_MWSRXFREQL_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSRXFREQH_RST    0x0
#define BT_MWSRXFREQL_RST    0xFFFF

__INLINE void bt_mwsrxtable0_pack(uint16_t mwsrxfreqh, uint16_t mwsrxfreql)
{
    ASSERT_ERR((((uint32_t)mwsrxfreqh << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsrxfreql << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSRXTABLE0_ADDR, ((uint32_t)mwsrxfreqh << 16) | ((uint32_t)mwsrxfreql << 0));
}

__INLINE void bt_mwsrxtable0_unpack(uint16_t *mwsrxfreqh, uint16_t *mwsrxfreql)
{
    uint32_t localVal = REG_BT_RD(BT_MWSRXTABLE0_ADDR);

    *mwsrxfreqh = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsrxfreql = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwsrxtable0_mwsrxfreqh_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSRXTABLE0_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwsrxtable0_mwsrxfreqh_setf(uint16_t mwsrxfreqh)
{
    ASSERT_ERR((((uint32_t)mwsrxfreqh << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSRXTABLE0_ADDR, (REG_BT_RD(BT_MWSRXTABLE0_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsrxfreqh << 16));
}

__INLINE uint16_t bt_mwsrxtable0_mwsrxfreql_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSRXTABLE0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwsrxtable0_mwsrxfreql_setf(uint16_t mwsrxfreql)
{
    ASSERT_ERR((((uint32_t)mwsrxfreql << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSRXTABLE0_ADDR, (REG_BT_RD(BT_MWSRXTABLE0_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsrxfreql << 0));
}

/**
 * @brief MWSSFTABLE1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH1   0x0
 *  15:00        MWSSCANFREQL1   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE1_ADDR   0x500905B8
#define BT_MWSSFTABLE1_OFFSET 0x000001B8
#define BT_MWSSFTABLE1_INDEX  0x0000006E
#define BT_MWSSFTABLE1_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable1_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE1_ADDR);
}

__INLINE void bt_mwssftable1_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE1_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH1_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH1_LSB    16
#define BT_MWSSCANFREQH1_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL1_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL1_LSB    0
#define BT_MWSSCANFREQL1_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH1_RST    0x0
#define BT_MWSSCANFREQL1_RST    0xFFFF

__INLINE void bt_mwssftable1_pack(uint16_t mwsscanfreqh1, uint16_t mwsscanfreql1)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE1_ADDR, ((uint32_t)mwsscanfreqh1 << 16) | ((uint32_t)mwsscanfreql1 << 0));
}

__INLINE void bt_mwssftable1_unpack(uint16_t *mwsscanfreqh1, uint16_t *mwsscanfreql1)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE1_ADDR);

    *mwsscanfreqh1 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql1 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable1_mwsscanfreqh1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE1_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable1_mwsscanfreqh1_setf(uint16_t mwsscanfreqh1)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE1_ADDR, (REG_BT_RD(BT_MWSSFTABLE1_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh1 << 16));
}

__INLINE uint16_t bt_mwssftable1_mwsscanfreql1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE1_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable1_mwsscanfreql1_setf(uint16_t mwsscanfreql1)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE1_ADDR, (REG_BT_RD(BT_MWSSFTABLE1_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql1 << 0));
}

/**
 * @brief MWSSFTABLE2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH2   0x0
 *  15:00        MWSSCANFREQL2   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE2_ADDR   0x500905BC
#define BT_MWSSFTABLE2_OFFSET 0x000001BC
#define BT_MWSSFTABLE2_INDEX  0x0000006F
#define BT_MWSSFTABLE2_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable2_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE2_ADDR);
}

__INLINE void bt_mwssftable2_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE2_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH2_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH2_LSB    16
#define BT_MWSSCANFREQH2_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL2_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL2_LSB    0
#define BT_MWSSCANFREQL2_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH2_RST    0x0
#define BT_MWSSCANFREQL2_RST    0xFFFF

__INLINE void bt_mwssftable2_pack(uint16_t mwsscanfreqh2, uint16_t mwsscanfreql2)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql2 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE2_ADDR, ((uint32_t)mwsscanfreqh2 << 16) | ((uint32_t)mwsscanfreql2 << 0));
}

__INLINE void bt_mwssftable2_unpack(uint16_t *mwsscanfreqh2, uint16_t *mwsscanfreql2)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE2_ADDR);

    *mwsscanfreqh2 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql2 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable2_mwsscanfreqh2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE2_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable2_mwsscanfreqh2_setf(uint16_t mwsscanfreqh2)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh2 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE2_ADDR, (REG_BT_RD(BT_MWSSFTABLE2_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh2 << 16));
}

__INLINE uint16_t bt_mwssftable2_mwsscanfreql2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE2_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable2_mwsscanfreql2_setf(uint16_t mwsscanfreql2)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql2 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE2_ADDR, (REG_BT_RD(BT_MWSSFTABLE2_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql2 << 0));
}

/**
 * @brief MWSSFTABLE3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH3   0x0
 *  15:00        MWSSCANFREQL3   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE3_ADDR   0x500905C0
#define BT_MWSSFTABLE3_OFFSET 0x000001C0
#define BT_MWSSFTABLE3_INDEX  0x00000070
#define BT_MWSSFTABLE3_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable3_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE3_ADDR);
}

__INLINE void bt_mwssftable3_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE3_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH3_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH3_LSB    16
#define BT_MWSSCANFREQH3_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL3_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL3_LSB    0
#define BT_MWSSCANFREQL3_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH3_RST    0x0
#define BT_MWSSCANFREQL3_RST    0xFFFF

__INLINE void bt_mwssftable3_pack(uint16_t mwsscanfreqh3, uint16_t mwsscanfreql3)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh3 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE3_ADDR, ((uint32_t)mwsscanfreqh3 << 16) | ((uint32_t)mwsscanfreql3 << 0));
}

__INLINE void bt_mwssftable3_unpack(uint16_t *mwsscanfreqh3, uint16_t *mwsscanfreql3)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE3_ADDR);

    *mwsscanfreqh3 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql3 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable3_mwsscanfreqh3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE3_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable3_mwsscanfreqh3_setf(uint16_t mwsscanfreqh3)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh3 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE3_ADDR, (REG_BT_RD(BT_MWSSFTABLE3_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh3 << 16));
}

__INLINE uint16_t bt_mwssftable3_mwsscanfreql3_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE3_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable3_mwsscanfreql3_setf(uint16_t mwsscanfreql3)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql3 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE3_ADDR, (REG_BT_RD(BT_MWSSFTABLE3_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql3 << 0));
}

/**
 * @brief MWSSFTABLE4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH4   0x0
 *  15:00        MWSSCANFREQL4   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE4_ADDR   0x500905C4
#define BT_MWSSFTABLE4_OFFSET 0x000001C4
#define BT_MWSSFTABLE4_INDEX  0x00000071
#define BT_MWSSFTABLE4_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable4_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE4_ADDR);
}

__INLINE void bt_mwssftable4_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE4_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH4_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH4_LSB    16
#define BT_MWSSCANFREQH4_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL4_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL4_LSB    0
#define BT_MWSSCANFREQL4_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH4_RST    0x0
#define BT_MWSSCANFREQL4_RST    0xFFFF

__INLINE void bt_mwssftable4_pack(uint16_t mwsscanfreqh4, uint16_t mwsscanfreql4)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql4 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE4_ADDR, ((uint32_t)mwsscanfreqh4 << 16) | ((uint32_t)mwsscanfreql4 << 0));
}

__INLINE void bt_mwssftable4_unpack(uint16_t *mwsscanfreqh4, uint16_t *mwsscanfreql4)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE4_ADDR);

    *mwsscanfreqh4 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql4 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable4_mwsscanfreqh4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE4_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable4_mwsscanfreqh4_setf(uint16_t mwsscanfreqh4)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh4 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE4_ADDR, (REG_BT_RD(BT_MWSSFTABLE4_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh4 << 16));
}

__INLINE uint16_t bt_mwssftable4_mwsscanfreql4_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE4_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable4_mwsscanfreql4_setf(uint16_t mwsscanfreql4)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql4 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE4_ADDR, (REG_BT_RD(BT_MWSSFTABLE4_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql4 << 0));
}

/**
 * @brief MWSSFTABLE5 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH5   0x0
 *  15:00        MWSSCANFREQL5   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE5_ADDR   0x500905C8
#define BT_MWSSFTABLE5_OFFSET 0x000001C8
#define BT_MWSSFTABLE5_INDEX  0x00000072
#define BT_MWSSFTABLE5_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable5_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE5_ADDR);
}

__INLINE void bt_mwssftable5_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE5_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH5_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH5_LSB    16
#define BT_MWSSCANFREQH5_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL5_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL5_LSB    0
#define BT_MWSSCANFREQL5_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH5_RST    0x0
#define BT_MWSSCANFREQL5_RST    0xFFFF

__INLINE void bt_mwssftable5_pack(uint16_t mwsscanfreqh5, uint16_t mwsscanfreql5)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh5 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql5 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE5_ADDR, ((uint32_t)mwsscanfreqh5 << 16) | ((uint32_t)mwsscanfreql5 << 0));
}

__INLINE void bt_mwssftable5_unpack(uint16_t *mwsscanfreqh5, uint16_t *mwsscanfreql5)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE5_ADDR);

    *mwsscanfreqh5 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql5 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable5_mwsscanfreqh5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE5_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable5_mwsscanfreqh5_setf(uint16_t mwsscanfreqh5)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh5 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE5_ADDR, (REG_BT_RD(BT_MWSSFTABLE5_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh5 << 16));
}

__INLINE uint16_t bt_mwssftable5_mwsscanfreql5_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE5_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable5_mwsscanfreql5_setf(uint16_t mwsscanfreql5)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql5 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE5_ADDR, (REG_BT_RD(BT_MWSSFTABLE5_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql5 << 0));
}

/**
 * @brief MWSSFTABLE6 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH6   0x0
 *  15:00        MWSSCANFREQL6   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE6_ADDR   0x500905CC
#define BT_MWSSFTABLE6_OFFSET 0x000001CC
#define BT_MWSSFTABLE6_INDEX  0x00000073
#define BT_MWSSFTABLE6_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable6_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE6_ADDR);
}

__INLINE void bt_mwssftable6_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE6_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH6_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH6_LSB    16
#define BT_MWSSCANFREQH6_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL6_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL6_LSB    0
#define BT_MWSSCANFREQL6_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH6_RST    0x0
#define BT_MWSSCANFREQL6_RST    0xFFFF

__INLINE void bt_mwssftable6_pack(uint16_t mwsscanfreqh6, uint16_t mwsscanfreql6)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh6 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql6 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE6_ADDR, ((uint32_t)mwsscanfreqh6 << 16) | ((uint32_t)mwsscanfreql6 << 0));
}

__INLINE void bt_mwssftable6_unpack(uint16_t *mwsscanfreqh6, uint16_t *mwsscanfreql6)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE6_ADDR);

    *mwsscanfreqh6 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql6 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable6_mwsscanfreqh6_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE6_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable6_mwsscanfreqh6_setf(uint16_t mwsscanfreqh6)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh6 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE6_ADDR, (REG_BT_RD(BT_MWSSFTABLE6_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh6 << 16));
}

__INLINE uint16_t bt_mwssftable6_mwsscanfreql6_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE6_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable6_mwsscanfreql6_setf(uint16_t mwsscanfreql6)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql6 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE6_ADDR, (REG_BT_RD(BT_MWSSFTABLE6_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql6 << 0));
}

/**
 * @brief MWSSFTABLE7 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH7   0x0
 *  15:00        MWSSCANFREQL7   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE7_ADDR   0x500905D0
#define BT_MWSSFTABLE7_OFFSET 0x000001D0
#define BT_MWSSFTABLE7_INDEX  0x00000074
#define BT_MWSSFTABLE7_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable7_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE7_ADDR);
}

__INLINE void bt_mwssftable7_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE7_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH7_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH7_LSB    16
#define BT_MWSSCANFREQH7_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL7_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL7_LSB    0
#define BT_MWSSCANFREQL7_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH7_RST    0x0
#define BT_MWSSCANFREQL7_RST    0xFFFF

__INLINE void bt_mwssftable7_pack(uint16_t mwsscanfreqh7, uint16_t mwsscanfreql7)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh7 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql7 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE7_ADDR, ((uint32_t)mwsscanfreqh7 << 16) | ((uint32_t)mwsscanfreql7 << 0));
}

__INLINE void bt_mwssftable7_unpack(uint16_t *mwsscanfreqh7, uint16_t *mwsscanfreql7)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE7_ADDR);

    *mwsscanfreqh7 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql7 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable7_mwsscanfreqh7_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE7_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable7_mwsscanfreqh7_setf(uint16_t mwsscanfreqh7)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh7 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE7_ADDR, (REG_BT_RD(BT_MWSSFTABLE7_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh7 << 16));
}

__INLINE uint16_t bt_mwssftable7_mwsscanfreql7_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE7_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable7_mwsscanfreql7_setf(uint16_t mwsscanfreql7)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql7 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE7_ADDR, (REG_BT_RD(BT_MWSSFTABLE7_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql7 << 0));
}

/**
 * @brief MWSSFTABLE8 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        MWSSCANFREQH8   0x0
 *  15:00        MWSSCANFREQL8   0xFFFF
 * </pre>
 */
#define BT_MWSSFTABLE8_ADDR   0x500905D4
#define BT_MWSSFTABLE8_OFFSET 0x000001D4
#define BT_MWSSFTABLE8_INDEX  0x00000075
#define BT_MWSSFTABLE8_RESET  0x0000FFFF

__INLINE uint32_t bt_mwssftable8_get(void)
{
    return REG_BT_RD(BT_MWSSFTABLE8_ADDR);
}

__INLINE void bt_mwssftable8_set(uint32_t value)
{
    REG_BT_WR(BT_MWSSFTABLE8_ADDR, value);
}

// field definitions
#define BT_MWSSCANFREQH8_MASK   ((uint32_t)0xFFFF0000)
#define BT_MWSSCANFREQH8_LSB    16
#define BT_MWSSCANFREQH8_WIDTH  ((uint32_t)0x00000010)
#define BT_MWSSCANFREQL8_MASK   ((uint32_t)0x0000FFFF)
#define BT_MWSSCANFREQL8_LSB    0
#define BT_MWSSCANFREQL8_WIDTH  ((uint32_t)0x00000010)

#define BT_MWSSCANFREQH8_RST    0x0
#define BT_MWSSCANFREQL8_RST    0xFFFF

__INLINE void bt_mwssftable8_pack(uint16_t mwsscanfreqh8, uint16_t mwsscanfreql8)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh8 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfreql8 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE8_ADDR, ((uint32_t)mwsscanfreqh8 << 16) | ((uint32_t)mwsscanfreql8 << 0));
}

__INLINE void bt_mwssftable8_unpack(uint16_t *mwsscanfreqh8, uint16_t *mwsscanfreql8)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE8_ADDR);

    *mwsscanfreqh8 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwsscanfreql8 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_mwssftable8_mwsscanfreqh8_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE8_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_mwssftable8_mwsscanfreqh8_setf(uint16_t mwsscanfreqh8)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqh8 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_MWSSFTABLE8_ADDR, (REG_BT_RD(BT_MWSSFTABLE8_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsscanfreqh8 << 16));
}

__INLINE uint16_t bt_mwssftable8_mwsscanfreql8_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSSFTABLE8_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_mwssftable8_mwsscanfreql8_setf(uint16_t mwsscanfreql8)
{
    ASSERT_ERR((((uint32_t)mwsscanfreql8 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_MWSSFTABLE8_ADDR, (REG_BT_RD(BT_MWSSFTABLE8_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwsscanfreql8 << 0));
}

/**
 * @brief MWSFRSYNCOFFSET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  14:00   MWSFRSYNCASSERTDLY   0x0
 * </pre>
 */
#define BT_MWSFRSYNCOFFSET_ADDR   0x500905E0
#define BT_MWSFRSYNCOFFSET_OFFSET 0x000001E0
#define BT_MWSFRSYNCOFFSET_INDEX  0x00000078
#define BT_MWSFRSYNCOFFSET_RESET  0x00000000

__INLINE uint32_t bt_mwsfrsyncoffset_get(void)
{
    return REG_BT_RD(BT_MWSFRSYNCOFFSET_ADDR);
}

__INLINE void bt_mwsfrsyncoffset_set(uint32_t value)
{
    REG_BT_WR(BT_MWSFRSYNCOFFSET_ADDR, value);
}

// field definitions
#define BT_MWSFRSYNCASSERTDLY_MASK   ((uint32_t)0x00007FFF)
#define BT_MWSFRSYNCASSERTDLY_LSB    0
#define BT_MWSFRSYNCASSERTDLY_WIDTH  ((uint32_t)0x0000000F)

#define BT_MWSFRSYNCASSERTDLY_RST    0x0

__INLINE uint16_t bt_mwsfrsyncoffset_mwsfrsyncassertdly_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSFRSYNCOFFSET_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00007FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_mwsfrsyncoffset_mwsfrsyncassertdly_setf(uint16_t mwsfrsyncassertdly)
{
    ASSERT_ERR((((uint32_t)mwsfrsyncassertdly << 0) & ~((uint32_t)0x00007FFF)) == 0);
    REG_BT_WR(BT_MWSFRSYNCOFFSET_ADDR, (uint32_t)mwsfrsyncassertdly << 0);
}

/**
 * @brief MWSTXOFFSET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16     MWSTXDEASSERTDLY   0x0
 *  10:00       MWSTXASSERTDLY   0x0
 * </pre>
 */
#define BT_MWSTXOFFSET_ADDR   0x500905E4
#define BT_MWSTXOFFSET_OFFSET 0x000001E4
#define BT_MWSTXOFFSET_INDEX  0x00000079
#define BT_MWSTXOFFSET_RESET  0x00000000

__INLINE uint32_t bt_mwstxoffset_get(void)
{
    return REG_BT_RD(BT_MWSTXOFFSET_ADDR);
}

__INLINE void bt_mwstxoffset_set(uint32_t value)
{
    REG_BT_WR(BT_MWSTXOFFSET_ADDR, value);
}

// field definitions
#define BT_MWSTXDEASSERTDLY_MASK   ((uint32_t)0x07FF0000)
#define BT_MWSTXDEASSERTDLY_LSB    16
#define BT_MWSTXDEASSERTDLY_WIDTH  ((uint32_t)0x0000000B)
#define BT_MWSTXASSERTDLY_MASK     ((uint32_t)0x000007FF)
#define BT_MWSTXASSERTDLY_LSB      0
#define BT_MWSTXASSERTDLY_WIDTH    ((uint32_t)0x0000000B)

#define BT_MWSTXDEASSERTDLY_RST    0x0
#define BT_MWSTXASSERTDLY_RST      0x0

__INLINE void bt_mwstxoffset_pack(uint16_t mwstxdeassertdly, uint16_t mwstxassertdly)
{
    ASSERT_ERR((((uint32_t)mwstxdeassertdly << 16) & ~((uint32_t)0x07FF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwstxassertdly << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_MWSTXOFFSET_ADDR, ((uint32_t)mwstxdeassertdly << 16) | ((uint32_t)mwstxassertdly << 0));
}

__INLINE void bt_mwstxoffset_unpack(uint16_t *mwstxdeassertdly, uint16_t *mwstxassertdly)
{
    uint32_t localVal = REG_BT_RD(BT_MWSTXOFFSET_ADDR);

    *mwstxdeassertdly = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *mwstxassertdly = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint16_t bt_mwstxoffset_mwstxdeassertdly_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSTXOFFSET_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE void bt_mwstxoffset_mwstxdeassertdly_setf(uint16_t mwstxdeassertdly)
{
    ASSERT_ERR((((uint32_t)mwstxdeassertdly << 16) & ~((uint32_t)0x07FF0000)) == 0);
    REG_BT_WR(BT_MWSTXOFFSET_ADDR, (REG_BT_RD(BT_MWSTXOFFSET_ADDR) & ~((uint32_t)0x07FF0000)) | ((uint32_t)mwstxdeassertdly << 16));
}

__INLINE uint16_t bt_mwstxoffset_mwstxassertdly_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSTXOFFSET_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

__INLINE void bt_mwstxoffset_mwstxassertdly_setf(uint16_t mwstxassertdly)
{
    ASSERT_ERR((((uint32_t)mwstxassertdly << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_MWSTXOFFSET_ADDR, (REG_BT_RD(BT_MWSTXOFFSET_ADDR) & ~((uint32_t)0x000007FF)) | ((uint32_t)mwstxassertdly << 0));
}

/**
 * @brief MWSRXOFFSET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16     MWSRXDEASSERTDLY   0x0
 *  10:00       MWSRXASSERTDLY   0x0
 * </pre>
 */
#define BT_MWSRXOFFSET_ADDR   0x500905E8
#define BT_MWSRXOFFSET_OFFSET 0x000001E8
#define BT_MWSRXOFFSET_INDEX  0x0000007A
#define BT_MWSRXOFFSET_RESET  0x00000000

__INLINE uint32_t bt_mwsrxoffset_get(void)
{
    return REG_BT_RD(BT_MWSRXOFFSET_ADDR);
}

__INLINE void bt_mwsrxoffset_set(uint32_t value)
{
    REG_BT_WR(BT_MWSRXOFFSET_ADDR, value);
}

// field definitions
#define BT_MWSRXDEASSERTDLY_MASK   ((uint32_t)0x07FF0000)
#define BT_MWSRXDEASSERTDLY_LSB    16
#define BT_MWSRXDEASSERTDLY_WIDTH  ((uint32_t)0x0000000B)
#define BT_MWSRXASSERTDLY_MASK     ((uint32_t)0x000007FF)
#define BT_MWSRXASSERTDLY_LSB      0
#define BT_MWSRXASSERTDLY_WIDTH    ((uint32_t)0x0000000B)

#define BT_MWSRXDEASSERTDLY_RST    0x0
#define BT_MWSRXASSERTDLY_RST      0x0

__INLINE void bt_mwsrxoffset_pack(uint16_t mwsrxdeassertdly, uint16_t mwsrxassertdly)
{
    ASSERT_ERR((((uint32_t)mwsrxdeassertdly << 16) & ~((uint32_t)0x07FF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwsrxassertdly << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_MWSRXOFFSET_ADDR, ((uint32_t)mwsrxdeassertdly << 16) | ((uint32_t)mwsrxassertdly << 0));
}

__INLINE void bt_mwsrxoffset_unpack(uint16_t *mwsrxdeassertdly, uint16_t *mwsrxassertdly)
{
    uint32_t localVal = REG_BT_RD(BT_MWSRXOFFSET_ADDR);

    *mwsrxdeassertdly = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *mwsrxassertdly = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint16_t bt_mwsrxoffset_mwsrxdeassertdly_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSRXOFFSET_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE void bt_mwsrxoffset_mwsrxdeassertdly_setf(uint16_t mwsrxdeassertdly)
{
    ASSERT_ERR((((uint32_t)mwsrxdeassertdly << 16) & ~((uint32_t)0x07FF0000)) == 0);
    REG_BT_WR(BT_MWSRXOFFSET_ADDR, (REG_BT_RD(BT_MWSRXOFFSET_ADDR) & ~((uint32_t)0x07FF0000)) | ((uint32_t)mwsrxdeassertdly << 16));
}

__INLINE uint16_t bt_mwsrxoffset_mwsrxassertdly_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSRXOFFSET_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

__INLINE void bt_mwsrxoffset_mwsrxassertdly_setf(uint16_t mwsrxassertdly)
{
    ASSERT_ERR((((uint32_t)mwsrxassertdly << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_MWSRXOFFSET_ADDR, (REG_BT_RD(BT_MWSRXOFFSET_ADDR) & ~((uint32_t)0x000007FF)) | ((uint32_t)mwsrxassertdly << 0));
}

/**
 * @brief MWSWCICNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:19      MWSWCI_RXINTDIV   0x0
 *  18:16    MWSWCI_RXFRACTDIV   0x0
 *  15:03      MWSWCI_TXINTDIV   0x0
 *  02:00    MWSWCI_TXFRACTDIV   0x0
 * </pre>
 */
#define BT_MWSWCICNTL0_ADDR   0x500905F0
#define BT_MWSWCICNTL0_OFFSET 0x000001F0
#define BT_MWSWCICNTL0_INDEX  0x0000007C
#define BT_MWSWCICNTL0_RESET  0x00000000

__INLINE uint32_t bt_mwswcicntl0_get(void)
{
    return REG_BT_RD(BT_MWSWCICNTL0_ADDR);
}

__INLINE void bt_mwswcicntl0_set(uint32_t value)
{
    REG_BT_WR(BT_MWSWCICNTL0_ADDR, value);
}

// field definitions
#define BT_MWSWCI_RXINTDIV_MASK     ((uint32_t)0xFFF80000)
#define BT_MWSWCI_RXINTDIV_LSB      19
#define BT_MWSWCI_RXINTDIV_WIDTH    ((uint32_t)0x0000000D)
#define BT_MWSWCI_RXFRACTDIV_MASK   ((uint32_t)0x00070000)
#define BT_MWSWCI_RXFRACTDIV_LSB    16
#define BT_MWSWCI_RXFRACTDIV_WIDTH  ((uint32_t)0x00000003)
#define BT_MWSWCI_TXINTDIV_MASK     ((uint32_t)0x0000FFF8)
#define BT_MWSWCI_TXINTDIV_LSB      3
#define BT_MWSWCI_TXINTDIV_WIDTH    ((uint32_t)0x0000000D)
#define BT_MWSWCI_TXFRACTDIV_MASK   ((uint32_t)0x00000007)
#define BT_MWSWCI_TXFRACTDIV_LSB    0
#define BT_MWSWCI_TXFRACTDIV_WIDTH  ((uint32_t)0x00000003)

#define BT_MWSWCI_RXINTDIV_RST      0x0
#define BT_MWSWCI_RXFRACTDIV_RST    0x0
#define BT_MWSWCI_TXINTDIV_RST      0x0
#define BT_MWSWCI_TXFRACTDIV_RST    0x0

__INLINE void bt_mwswcicntl0_pack(uint16_t mwswcirxintdiv, uint8_t mwswcirxfractdiv, uint16_t mwswcitxintdiv, uint8_t mwswcitxfractdiv)
{
    ASSERT_ERR((((uint32_t)mwswcirxintdiv << 19) & ~((uint32_t)0xFFF80000)) == 0);
    ASSERT_ERR((((uint32_t)mwswcirxfractdiv << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)mwswcitxintdiv << 3) & ~((uint32_t)0x0000FFF8)) == 0);
    ASSERT_ERR((((uint32_t)mwswcitxfractdiv << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSWCICNTL0_ADDR, ((uint32_t)mwswcirxintdiv << 19) | ((uint32_t)mwswcirxfractdiv << 16) | ((uint32_t)mwswcitxintdiv << 3) | ((uint32_t)mwswcitxfractdiv << 0));
}

__INLINE void bt_mwswcicntl0_unpack(uint16_t *mwswcirxintdiv, uint8_t *mwswcirxfractdiv, uint16_t *mwswcitxintdiv, uint8_t *mwswcitxfractdiv)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL0_ADDR);

    *mwswcirxintdiv = (localVal & ((uint32_t)0xFFF80000)) >> 19;
    *mwswcirxfractdiv = (localVal & ((uint32_t)0x00070000)) >> 16;
    *mwswcitxintdiv = (localVal & ((uint32_t)0x0000FFF8)) >> 3;
    *mwswcitxfractdiv = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint16_t bt_mwswcicntl0_mwswci_rxintdiv_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL0_ADDR);
    return ((localVal & ((uint32_t)0xFFF80000)) >> 19);
}

__INLINE void bt_mwswcicntl0_mwswci_rxintdiv_setf(uint16_t mwswcirxintdiv)
{
    ASSERT_ERR((((uint32_t)mwswcirxintdiv << 19) & ~((uint32_t)0xFFF80000)) == 0);
    REG_BT_WR(BT_MWSWCICNTL0_ADDR, (REG_BT_RD(BT_MWSWCICNTL0_ADDR) & ~((uint32_t)0xFFF80000)) | ((uint32_t)mwswcirxintdiv << 19));
}

__INLINE uint8_t bt_mwswcicntl0_mwswci_rxfractdiv_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void bt_mwswcicntl0_mwswci_rxfractdiv_setf(uint8_t mwswcirxfractdiv)
{
    ASSERT_ERR((((uint32_t)mwswcirxfractdiv << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_BT_WR(BT_MWSWCICNTL0_ADDR, (REG_BT_RD(BT_MWSWCICNTL0_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)mwswcirxfractdiv << 16));
}

__INLINE uint16_t bt_mwswcicntl0_mwswci_txintdiv_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFF8)) >> 3);
}

__INLINE void bt_mwswcicntl0_mwswci_txintdiv_setf(uint16_t mwswcitxintdiv)
{
    ASSERT_ERR((((uint32_t)mwswcitxintdiv << 3) & ~((uint32_t)0x0000FFF8)) == 0);
    REG_BT_WR(BT_MWSWCICNTL0_ADDR, (REG_BT_RD(BT_MWSWCICNTL0_ADDR) & ~((uint32_t)0x0000FFF8)) | ((uint32_t)mwswcitxintdiv << 3));
}

__INLINE uint8_t bt_mwswcicntl0_mwswci_txfractdiv_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_mwswcicntl0_mwswci_txfractdiv_setf(uint8_t mwswcitxfractdiv)
{
    ASSERT_ERR((((uint32_t)mwswcitxfractdiv << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_MWSWCICNTL0_ADDR, (REG_BT_RD(BT_MWSWCICNTL0_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)mwswcitxfractdiv << 0));
}

/**
 * @brief MWSWCICNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31     RESEND_REAL_TIME   0
 *     30           SENDWCIMSG   0
 *     15               WCISEL   0
 *  03:00            WCIIFSCNT   0x0
 * </pre>
 */
#define BT_MWSWCICNTL1_ADDR   0x500905F4
#define BT_MWSWCICNTL1_OFFSET 0x000001F4
#define BT_MWSWCICNTL1_INDEX  0x0000007D
#define BT_MWSWCICNTL1_RESET  0x00000000

__INLINE uint32_t bt_mwswcicntl1_get(void)
{
    return REG_BT_RD(BT_MWSWCICNTL1_ADDR);
}

__INLINE void bt_mwswcicntl1_set(uint32_t value)
{
    REG_BT_WR(BT_MWSWCICNTL1_ADDR, value);
}

// field definitions
#define BT_RESEND_REAL_TIME_BIT    ((uint32_t)0x80000000)
#define BT_RESEND_REAL_TIME_POS    31
#define BT_SENDWCIMSG_BIT          ((uint32_t)0x40000000)
#define BT_SENDWCIMSG_POS          30
#define BT_WCISEL_BIT              ((uint32_t)0x00008000)
#define BT_WCISEL_POS              15
#define BT_WCIIFSCNT_MASK          ((uint32_t)0x0000000F)
#define BT_WCIIFSCNT_LSB           0
#define BT_WCIIFSCNT_WIDTH         ((uint32_t)0x00000004)

#define BT_RESEND_REAL_TIME_RST    0x0
#define BT_SENDWCIMSG_RST          0x0
#define BT_WCISEL_RST              0x0
#define BT_WCIIFSCNT_RST           0x0

__INLINE void bt_mwswcicntl1_pack(uint8_t resendrealtime, uint8_t sendwcimsg, uint8_t wcisel, uint8_t wciifscnt)
{
    ASSERT_ERR((((uint32_t)resendrealtime << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)sendwcimsg << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)wcisel << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)wciifscnt << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_MWSWCICNTL1_ADDR, ((uint32_t)resendrealtime << 31) | ((uint32_t)sendwcimsg << 30) | ((uint32_t)wcisel << 15) | ((uint32_t)wciifscnt << 0));
}

__INLINE void bt_mwswcicntl1_unpack(uint8_t *resendrealtime, uint8_t *sendwcimsg, uint8_t *wcisel, uint8_t *wciifscnt)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL1_ADDR);

    *resendrealtime = (localVal & ((uint32_t)0x80000000)) >> 31;
    *sendwcimsg = (localVal & ((uint32_t)0x40000000)) >> 30;
    *wcisel = (localVal & ((uint32_t)0x00008000)) >> 15;
    *wciifscnt = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_mwswcicntl1_resend_real_time_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_mwswcicntl1_resend_real_time_setf(uint8_t resendrealtime)
{
    ASSERT_ERR((((uint32_t)resendrealtime << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_MWSWCICNTL1_ADDR, (REG_BT_RD(BT_MWSWCICNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)resendrealtime << 31));
}

__INLINE uint8_t bt_mwswcicntl1_sendwcimsg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void bt_mwswcicntl1_sendwcimsg_setf(uint8_t sendwcimsg)
{
    ASSERT_ERR((((uint32_t)sendwcimsg << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BT_WR(BT_MWSWCICNTL1_ADDR, (REG_BT_RD(BT_MWSWCICNTL1_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)sendwcimsg << 30));
}

__INLINE uint8_t bt_mwswcicntl1_wcisel_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_mwswcicntl1_wcisel_setf(uint8_t wcisel)
{
    ASSERT_ERR((((uint32_t)wcisel << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_MWSWCICNTL1_ADDR, (REG_BT_RD(BT_MWSWCICNTL1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)wcisel << 15));
}

__INLINE uint8_t bt_mwswcicntl1_wciifscnt_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_mwswcicntl1_wciifscnt_setf(uint8_t wciifscnt)
{
    ASSERT_ERR((((uint32_t)wciifscnt << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_MWSWCICNTL1_ADDR, (REG_BT_RD(BT_MWSWCICNTL1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)wciifscnt << 0));
}

/**
 * @brief MWSWCITXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:16              WCITXNB   0x0
 *  13:00             WCITXPTR   0x0
 * </pre>
 */
#define BT_MWSWCITXCNTL_ADDR   0x500905F8
#define BT_MWSWCITXCNTL_OFFSET 0x000001F8
#define BT_MWSWCITXCNTL_INDEX  0x0000007E
#define BT_MWSWCITXCNTL_RESET  0x00000000

__INLINE uint32_t bt_mwswcitxcntl_get(void)
{
    return REG_BT_RD(BT_MWSWCITXCNTL_ADDR);
}

__INLINE void bt_mwswcitxcntl_set(uint32_t value)
{
    REG_BT_WR(BT_MWSWCITXCNTL_ADDR, value);
}

// field definitions
#define BT_WCITXNB_MASK    ((uint32_t)0x003F0000)
#define BT_WCITXNB_LSB     16
#define BT_WCITXNB_WIDTH   ((uint32_t)0x00000006)
#define BT_WCITXPTR_MASK   ((uint32_t)0x00003FFF)
#define BT_WCITXPTR_LSB    0
#define BT_WCITXPTR_WIDTH  ((uint32_t)0x0000000E)

#define BT_WCITXNB_RST     0x0
#define BT_WCITXPTR_RST    0x0

__INLINE void bt_mwswcitxcntl_pack(uint8_t wcitxnb, uint16_t wcitxptr)
{
    ASSERT_ERR((((uint32_t)wcitxnb << 16) & ~((uint32_t)0x003F0000)) == 0);
    ASSERT_ERR((((uint32_t)wcitxptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_MWSWCITXCNTL_ADDR, ((uint32_t)wcitxnb << 16) | ((uint32_t)wcitxptr << 0));
}

__INLINE void bt_mwswcitxcntl_unpack(uint8_t *wcitxnb, uint16_t *wcitxptr)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCITXCNTL_ADDR);

    *wcitxnb = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *wcitxptr = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t bt_mwswcitxcntl_wcitxnb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCITXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE void bt_mwswcitxcntl_wcitxnb_setf(uint8_t wcitxnb)
{
    ASSERT_ERR((((uint32_t)wcitxnb << 16) & ~((uint32_t)0x003F0000)) == 0);
    REG_BT_WR(BT_MWSWCITXCNTL_ADDR, (REG_BT_RD(BT_MWSWCITXCNTL_ADDR) & ~((uint32_t)0x003F0000)) | ((uint32_t)wcitxnb << 16));
}

__INLINE uint16_t bt_mwswcitxcntl_wcitxptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCITXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_mwswcitxcntl_wcitxptr_setf(uint16_t wcitxptr)
{
    ASSERT_ERR((((uint32_t)wcitxptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_MWSWCITXCNTL_ADDR, (REG_BT_RD(BT_MWSWCITXCNTL_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)wcitxptr << 0));
}

/**
 * @brief MWSWCIRXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:16              WCIRXNB   0x0
 *  13:00             WCIRXPTR   0x0
 * </pre>
 */
#define BT_MWSWCIRXCNTL_ADDR   0x500905FC
#define BT_MWSWCIRXCNTL_OFFSET 0x000001FC
#define BT_MWSWCIRXCNTL_INDEX  0x0000007F
#define BT_MWSWCIRXCNTL_RESET  0x00000000

__INLINE uint32_t bt_mwswcirxcntl_get(void)
{
    return REG_BT_RD(BT_MWSWCIRXCNTL_ADDR);
}

__INLINE void bt_mwswcirxcntl_set(uint32_t value)
{
    REG_BT_WR(BT_MWSWCIRXCNTL_ADDR, value);
}

// field definitions
#define BT_WCIRXNB_MASK    ((uint32_t)0x003F0000)
#define BT_WCIRXNB_LSB     16
#define BT_WCIRXNB_WIDTH   ((uint32_t)0x00000006)
#define BT_WCIRXPTR_MASK   ((uint32_t)0x00003FFF)
#define BT_WCIRXPTR_LSB    0
#define BT_WCIRXPTR_WIDTH  ((uint32_t)0x0000000E)

#define BT_WCIRXNB_RST     0x0
#define BT_WCIRXPTR_RST    0x0

__INLINE void bt_mwswcirxcntl_pack(uint8_t wcirxnb, uint16_t wcirxptr)
{
    ASSERT_ERR((((uint32_t)wcirxnb << 16) & ~((uint32_t)0x003F0000)) == 0);
    ASSERT_ERR((((uint32_t)wcirxptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_MWSWCIRXCNTL_ADDR, ((uint32_t)wcirxnb << 16) | ((uint32_t)wcirxptr << 0));
}

__INLINE void bt_mwswcirxcntl_unpack(uint8_t *wcirxnb, uint16_t *wcirxptr)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCIRXCNTL_ADDR);

    *wcirxnb = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *wcirxptr = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t bt_mwswcirxcntl_wcirxnb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCIRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE void bt_mwswcirxcntl_wcirxnb_setf(uint8_t wcirxnb)
{
    ASSERT_ERR((((uint32_t)wcirxnb << 16) & ~((uint32_t)0x003F0000)) == 0);
    REG_BT_WR(BT_MWSWCIRXCNTL_ADDR, (REG_BT_RD(BT_MWSWCIRXCNTL_ADDR) & ~((uint32_t)0x003F0000)) | ((uint32_t)wcirxnb << 16));
}

__INLINE uint16_t bt_mwswcirxcntl_wcirxptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_MWSWCIRXCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_mwswcirxcntl_wcirxptr_setf(uint16_t wcirxptr)
{
    ASSERT_ERR((((uint32_t)wcirxptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_MWSWCIRXCNTL_ADDR, (REG_BT_RD(BT_MWSWCIRXCNTL_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)wcirxptr << 0));
}

/**
 * @brief LSAMCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         LOCAL_SAM_EN   0
 *  21:16     LOCAL_SAM_LENGTH   0x0
 *  13:00        LOCAL_SAM_PTR   0x0
 * </pre>
 */
#define BT_LSAMCNTL0_ADDR   0x50090600
#define BT_LSAMCNTL0_OFFSET 0x00000200
#define BT_LSAMCNTL0_INDEX  0x00000080
#define BT_LSAMCNTL0_RESET  0x00000000

__INLINE uint32_t bt_lsamcntl0_get(void)
{
    return REG_BT_RD(BT_LSAMCNTL0_ADDR);
}

__INLINE void bt_lsamcntl0_set(uint32_t value)
{
    REG_BT_WR(BT_LSAMCNTL0_ADDR, value);
}

// field definitions
#define BT_LOCAL_SAM_EN_BIT        ((uint32_t)0x80000000)
#define BT_LOCAL_SAM_EN_POS        31
#define BT_LOCAL_SAM_LENGTH_MASK   ((uint32_t)0x003F0000)
#define BT_LOCAL_SAM_LENGTH_LSB    16
#define BT_LOCAL_SAM_LENGTH_WIDTH  ((uint32_t)0x00000006)
#define BT_LOCAL_SAM_PTR_MASK      ((uint32_t)0x00003FFF)
#define BT_LOCAL_SAM_PTR_LSB       0
#define BT_LOCAL_SAM_PTR_WIDTH     ((uint32_t)0x0000000E)

#define BT_LOCAL_SAM_EN_RST        0x0
#define BT_LOCAL_SAM_LENGTH_RST    0x0
#define BT_LOCAL_SAM_PTR_RST       0x0

__INLINE void bt_lsamcntl0_pack(uint8_t localsamen, uint8_t localsamlength, uint16_t localsamptr)
{
    ASSERT_ERR((((uint32_t)localsamen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)localsamlength << 16) & ~((uint32_t)0x003F0000)) == 0);
    ASSERT_ERR((((uint32_t)localsamptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_LSAMCNTL0_ADDR, ((uint32_t)localsamen << 31) | ((uint32_t)localsamlength << 16) | ((uint32_t)localsamptr << 0));
}

__INLINE void bt_lsamcntl0_unpack(uint8_t *localsamen, uint8_t *localsamlength, uint16_t *localsamptr)
{
    uint32_t localVal = REG_BT_RD(BT_LSAMCNTL0_ADDR);

    *localsamen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *localsamlength = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *localsamptr = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t bt_lsamcntl0_local_sam_en_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_LSAMCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_lsamcntl0_local_sam_en_setf(uint8_t localsamen)
{
    ASSERT_ERR((((uint32_t)localsamen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_LSAMCNTL0_ADDR, (REG_BT_RD(BT_LSAMCNTL0_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)localsamen << 31));
}

__INLINE uint8_t bt_lsamcntl0_local_sam_length_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_LSAMCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE void bt_lsamcntl0_local_sam_length_setf(uint8_t localsamlength)
{
    ASSERT_ERR((((uint32_t)localsamlength << 16) & ~((uint32_t)0x003F0000)) == 0);
    REG_BT_WR(BT_LSAMCNTL0_ADDR, (REG_BT_RD(BT_LSAMCNTL0_ADDR) & ~((uint32_t)0x003F0000)) | ((uint32_t)localsamlength << 16));
}

__INLINE uint16_t bt_lsamcntl0_local_sam_ptr_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_LSAMCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_lsamcntl0_local_sam_ptr_setf(uint16_t localsamptr)
{
    ASSERT_ERR((((uint32_t)localsamptr << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_LSAMCNTL0_ADDR, (REG_BT_RD(BT_LSAMCNTL0_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)localsamptr << 0));
}

/**
 * @brief LSAMCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  05:00     LOCAL_SAM_OFFSET   0x0
 * </pre>
 */
#define BT_LSAMCNTL1_ADDR   0x50090604
#define BT_LSAMCNTL1_OFFSET 0x00000204
#define BT_LSAMCNTL1_INDEX  0x00000081
#define BT_LSAMCNTL1_RESET  0x00000000

__INLINE uint32_t bt_lsamcntl1_get(void)
{
    return REG_BT_RD(BT_LSAMCNTL1_ADDR);
}

__INLINE void bt_lsamcntl1_set(uint32_t value)
{
    REG_BT_WR(BT_LSAMCNTL1_ADDR, value);
}

// field definitions
#define BT_LOCAL_SAM_OFFSET_MASK   ((uint32_t)0x0000003F)
#define BT_LOCAL_SAM_OFFSET_LSB    0
#define BT_LOCAL_SAM_OFFSET_WIDTH  ((uint32_t)0x00000006)

#define BT_LOCAL_SAM_OFFSET_RST    0x0

__INLINE uint8_t bt_lsamcntl1_local_sam_offset_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_LSAMCNTL1_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000003F)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_lsamcntl1_local_sam_offset_setf(uint8_t localsamoffset)
{
    ASSERT_ERR((((uint32_t)localsamoffset << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_BT_WR(BT_LSAMCNTL1_ADDR, (uint32_t)localsamoffset << 0);
}

/**
 * @brief E_SCOCHANCNTL_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                 TOG0   0
 *     15        eSCOCHANSWEN0   0
 *     14          eSCOCHANEN0   0
 *     13              ITMODE0   0
 *  12:08            INTDELAY0   0x0
 *  07:00               TeSCO0   0x0
 * </pre>
 */
#define BT_E_SCOCHANCNTL_0_ADDR   0x50090610
#define BT_E_SCOCHANCNTL_0_OFFSET 0x00000210
#define BT_E_SCOCHANCNTL_0_INDEX  0x00000084
#define BT_E_SCOCHANCNTL_0_RESET  0x00000000

__INLINE uint32_t bt_e_scochancntl_0_get(void)
{
    return REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);
}

__INLINE void bt_e_scochancntl_0_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCHANCNTL_0_ADDR, value);
}

// field definitions
#define BT_TOG0_BIT               ((uint32_t)0x80000000)
#define BT_TOG0_POS               31
#define BT_E_SCOCHANSWEN_0_BIT    ((uint32_t)0x00008000)
#define BT_E_SCOCHANSWEN_0_POS    15
#define BT_E_SCOCHANEN_0_BIT      ((uint32_t)0x00004000)
#define BT_E_SCOCHANEN_0_POS      14
#define BT_ITMODE0_BIT            ((uint32_t)0x00002000)
#define BT_ITMODE0_POS            13
#define BT_INTDELAY0_MASK         ((uint32_t)0x00001F00)
#define BT_INTDELAY0_LSB          8
#define BT_INTDELAY0_WIDTH        ((uint32_t)0x00000005)
#define BT_TE_SCO_0_MASK          ((uint32_t)0x000000FF)
#define BT_TE_SCO_0_LSB           0
#define BT_TE_SCO_0_WIDTH         ((uint32_t)0x00000008)

#define BT_TOG0_RST               0x0
#define BT_E_SCOCHANSWEN_0_RST    0x0
#define BT_E_SCOCHANEN_0_RST      0x0
#define BT_ITMODE0_RST            0x0
#define BT_INTDELAY0_RST          0x0
#define BT_TE_SCO_0_RST           0x0

__INLINE void bt_e_scochancntl_0_pack(uint8_t escochanswen0, uint8_t escochanen0, uint8_t itmode0, uint8_t intdelay0, uint8_t tesco0)
{
    ASSERT_ERR((((uint32_t)escochanswen0 << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)escochanen0 << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)itmode0 << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)intdelay0 << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)tesco0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_0_ADDR, ((uint32_t)escochanswen0 << 15) | ((uint32_t)escochanen0 << 14) | ((uint32_t)itmode0 << 13) | ((uint32_t)intdelay0 << 8) | ((uint32_t)tesco0 << 0));
}

__INLINE void bt_e_scochancntl_0_unpack(uint8_t *tog0, uint8_t *escochanswen0, uint8_t *escochanen0, uint8_t *itmode0, uint8_t *intdelay0, uint8_t *tesco0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);

    *tog0 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *escochanswen0 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *escochanen0 = (localVal & ((uint32_t)0x00004000)) >> 14;
    *itmode0 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *intdelay0 = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *tesco0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bt_e_scochancntl_0_tog0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t bt_e_scochancntl_0_e_scochanswen_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_e_scochancntl_0_e_scochanswen_0_setf(uint8_t escochanswen0)
{
    ASSERT_ERR((((uint32_t)escochanswen0 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)escochanswen0 << 15));
}

__INLINE uint8_t bt_e_scochancntl_0_e_scochanen_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_e_scochancntl_0_e_scochanen_0_setf(uint8_t escochanen0)
{
    ASSERT_ERR((((uint32_t)escochanen0 << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)escochanen0 << 14));
}

__INLINE uint8_t bt_e_scochancntl_0_itmode0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_e_scochancntl_0_itmode0_setf(uint8_t itmode0)
{
    ASSERT_ERR((((uint32_t)itmode0 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)itmode0 << 13));
}

__INLINE uint8_t bt_e_scochancntl_0_intdelay0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void bt_e_scochancntl_0_intdelay0_setf(uint8_t intdelay0)
{
    ASSERT_ERR((((uint32_t)intdelay0 << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)intdelay0 << 8));
}

__INLINE uint8_t bt_e_scochancntl_0_te_sco_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bt_e_scochancntl_0_te_sco_0_setf(uint8_t tesco0)
{
    ASSERT_ERR((((uint32_t)tesco0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_0_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)tesco0 << 0));
}

/**
 * @brief E_SCOMUTECNTL_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     23           MUTE_SINK0   0
 *     22         MUTE_SOURCE0   0
 *  19:18              INVL0_1   0x2
 *  17:16              INVL0_0   0x2
 *  15:00            MUTEPATT0   0x0
 * </pre>
 */
#define BT_E_SCOMUTECNTL_0_ADDR   0x50090614
#define BT_E_SCOMUTECNTL_0_OFFSET 0x00000214
#define BT_E_SCOMUTECNTL_0_INDEX  0x00000085
#define BT_E_SCOMUTECNTL_0_RESET  0x000A0000

__INLINE uint32_t bt_e_scomutecntl_0_get(void)
{
    return REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR);
}

__INLINE void bt_e_scomutecntl_0_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOMUTECNTL_0_ADDR, value);
}

// field definitions
#define BT_MUTE_SINK0_BIT      ((uint32_t)0x00800000)
#define BT_MUTE_SINK0_POS      23
#define BT_MUTE_SOURCE0_BIT    ((uint32_t)0x00400000)
#define BT_MUTE_SOURCE0_POS    22
#define BT_INVL0_1_MASK        ((uint32_t)0x000C0000)
#define BT_INVL0_1_LSB         18
#define BT_INVL0_1_WIDTH       ((uint32_t)0x00000002)
#define BT_INVL0_0_MASK        ((uint32_t)0x00030000)
#define BT_INVL0_0_LSB         16
#define BT_INVL0_0_WIDTH       ((uint32_t)0x00000002)
#define BT_MUTEPATT0_MASK      ((uint32_t)0x0000FFFF)
#define BT_MUTEPATT0_LSB       0
#define BT_MUTEPATT0_WIDTH     ((uint32_t)0x00000010)

#define BT_MUTE_SINK0_RST      0x0
#define BT_MUTE_SOURCE0_RST    0x0
#define BT_INVL0_1_RST         0x2
#define BT_INVL0_0_RST         0x2
#define BT_MUTEPATT0_RST       0x0

__INLINE void bt_e_scomutecntl_0_pack(uint8_t mutesink0, uint8_t mutesource0, uint8_t invl01, uint8_t invl00, uint16_t mutepatt0)
{
    ASSERT_ERR((((uint32_t)mutesink0 << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)mutesource0 << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)invl01 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)invl00 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mutepatt0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_0_ADDR, ((uint32_t)mutesink0 << 23) | ((uint32_t)mutesource0 << 22) | ((uint32_t)invl01 << 18) | ((uint32_t)invl00 << 16) | ((uint32_t)mutepatt0 << 0));
}

__INLINE void bt_e_scomutecntl_0_unpack(uint8_t *mutesink0, uint8_t *mutesource0, uint8_t *invl01, uint8_t *invl00, uint16_t *mutepatt0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR);

    *mutesink0 = (localVal & ((uint32_t)0x00800000)) >> 23;
    *mutesource0 = (localVal & ((uint32_t)0x00400000)) >> 22;
    *invl01 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *invl00 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mutepatt0 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t bt_e_scomutecntl_0_mute_sink0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bt_e_scomutecntl_0_mute_sink0_setf(uint8_t mutesink0)
{
    ASSERT_ERR((((uint32_t)mutesink0 << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_0_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)mutesink0 << 23));
}

__INLINE uint8_t bt_e_scomutecntl_0_mute_source0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bt_e_scomutecntl_0_mute_source0_setf(uint8_t mutesource0)
{
    ASSERT_ERR((((uint32_t)mutesource0 << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_0_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)mutesource0 << 22));
}

__INLINE uint8_t bt_e_scomutecntl_0_invl0_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void bt_e_scomutecntl_0_invl0_1_setf(uint8_t invl01)
{
    ASSERT_ERR((((uint32_t)invl01 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_0_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)invl01 << 18));
}

__INLINE uint8_t bt_e_scomutecntl_0_invl0_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_e_scomutecntl_0_invl0_0_setf(uint8_t invl00)
{
    ASSERT_ERR((((uint32_t)invl00 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_0_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)invl00 << 16));
}

__INLINE uint16_t bt_e_scomutecntl_0_mutepatt0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_e_scomutecntl_0_mutepatt0_setf(uint16_t mutepatt0)
{
    ASSERT_ERR((((uint32_t)mutepatt0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_0_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_0_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mutepatt0 << 0));
}

/**
 * @brief E_SCOCURRENTTXPTR_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          eSCO0PTRTX1   0x0
 *  13:00          eSCO0PTRTX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTTXPTR_0_ADDR   0x50090618
#define BT_E_SCOCURRENTTXPTR_0_OFFSET 0x00000218
#define BT_E_SCOCURRENTTXPTR_0_INDEX  0x00000086
#define BT_E_SCOCURRENTTXPTR_0_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrenttxptr_0_get(void)
{
    return REG_BT_RD(BT_E_SCOCURRENTTXPTR_0_ADDR);
}

__INLINE void bt_e_scocurrenttxptr_0_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_0_ADDR, value);
}

// field definitions
#define BT_E_SCO_0PTRTX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCO_0PTRTX_1_LSB    16
#define BT_E_SCO_0PTRTX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCO_0PTRTX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCO_0PTRTX_0_LSB    0
#define BT_E_SCO_0PTRTX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCO_0PTRTX_1_RST    0x0
#define BT_E_SCO_0PTRTX_0_RST    0x0

__INLINE void bt_e_scocurrenttxptr_0_pack(uint16_t esco0ptrtx1, uint16_t esco0ptrtx0)
{
    ASSERT_ERR((((uint32_t)esco0ptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)esco0ptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_0_ADDR, ((uint32_t)esco0ptrtx1 << 16) | ((uint32_t)esco0ptrtx0 << 0));
}

__INLINE void bt_e_scocurrenttxptr_0_unpack(uint16_t *esco0ptrtx1, uint16_t *esco0ptrtx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_0_ADDR);

    *esco0ptrtx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *esco0ptrtx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrenttxptr_0_e_sco_0ptrtx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_0_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrenttxptr_0_e_sco_0ptrtx_1_setf(uint16_t esco0ptrtx1)
{
    ASSERT_ERR((((uint32_t)esco0ptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_0_ADDR, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_0_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)esco0ptrtx1 << 16));
}

__INLINE uint16_t bt_e_scocurrenttxptr_0_e_sco_0ptrtx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_0_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrenttxptr_0_e_sco_0ptrtx_0_setf(uint16_t esco0ptrtx0)
{
    ASSERT_ERR((((uint32_t)esco0ptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_0_ADDR, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_0_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)esco0ptrtx0 << 0));
}

/**
 * @brief E_SCOCURRENTRXPTR_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          eSCO0PTRRX1   0x0
 *  13:00          eSCO0PTRRX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTRXPTR_0_ADDR   0x5009061C
#define BT_E_SCOCURRENTRXPTR_0_OFFSET 0x0000021C
#define BT_E_SCOCURRENTRXPTR_0_INDEX  0x00000087
#define BT_E_SCOCURRENTRXPTR_0_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrentrxptr_0_get(void)
{
    return REG_BT_RD(BT_E_SCOCURRENTRXPTR_0_ADDR);
}

__INLINE void bt_e_scocurrentrxptr_0_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_0_ADDR, value);
}

// field definitions
#define BT_E_SCO_0PTRRX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCO_0PTRRX_1_LSB    16
#define BT_E_SCO_0PTRRX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCO_0PTRRX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCO_0PTRRX_0_LSB    0
#define BT_E_SCO_0PTRRX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCO_0PTRRX_1_RST    0x0
#define BT_E_SCO_0PTRRX_0_RST    0x0

__INLINE void bt_e_scocurrentrxptr_0_pack(uint16_t esco0ptrrx1, uint16_t esco0ptrrx0)
{
    ASSERT_ERR((((uint32_t)esco0ptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)esco0ptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_0_ADDR, ((uint32_t)esco0ptrrx1 << 16) | ((uint32_t)esco0ptrrx0 << 0));
}

__INLINE void bt_e_scocurrentrxptr_0_unpack(uint16_t *esco0ptrrx1, uint16_t *esco0ptrrx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_0_ADDR);

    *esco0ptrrx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *esco0ptrrx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrentrxptr_0_e_sco_0ptrrx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_0_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrentrxptr_0_e_sco_0ptrrx_1_setf(uint16_t esco0ptrrx1)
{
    ASSERT_ERR((((uint32_t)esco0ptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_0_ADDR, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_0_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)esco0ptrrx1 << 16));
}

__INLINE uint16_t bt_e_scocurrentrxptr_0_e_sco_0ptrrx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_0_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrentrxptr_0_e_sco_0ptrrx_0_setf(uint16_t esco0ptrrx0)
{
    ASSERT_ERR((((uint32_t)esco0ptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_0_ADDR, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_0_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)esco0ptrrx0 << 0));
}

/**
 * @brief E_SCOLTCNTL_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16              RETXNB0   0x1
 *     05           eSCOEDRRX0   0
 *     04           eSCOEDRTX0   0
 *     03             SYNTYPE0   0
 *  02:00           SYNLTADDR0   0x0
 * </pre>
 */
#define BT_E_SCOLTCNTL_0_ADDR   0x50090620
#define BT_E_SCOLTCNTL_0_OFFSET 0x00000220
#define BT_E_SCOLTCNTL_0_INDEX  0x00000088
#define BT_E_SCOLTCNTL_0_RESET  0x00010000

__INLINE uint32_t bt_e_scoltcntl_0_get(void)
{
    return REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR);
}

__INLINE void bt_e_scoltcntl_0_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOLTCNTL_0_ADDR, value);
}

// field definitions
#define BT_RETXNB0_MASK        ((uint32_t)0x00FF0000)
#define BT_RETXNB0_LSB         16
#define BT_RETXNB0_WIDTH       ((uint32_t)0x00000008)
#define BT_E_SCOEDRRX_0_BIT    ((uint32_t)0x00000020)
#define BT_E_SCOEDRRX_0_POS    5
#define BT_E_SCOEDRTX_0_BIT    ((uint32_t)0x00000010)
#define BT_E_SCOEDRTX_0_POS    4
#define BT_SYNTYPE0_BIT        ((uint32_t)0x00000008)
#define BT_SYNTYPE0_POS        3
#define BT_SYNLTADDR0_MASK     ((uint32_t)0x00000007)
#define BT_SYNLTADDR0_LSB      0
#define BT_SYNLTADDR0_WIDTH    ((uint32_t)0x00000003)

#define BT_RETXNB0_RST         0x1
#define BT_E_SCOEDRRX_0_RST    0x0
#define BT_E_SCOEDRTX_0_RST    0x0
#define BT_SYNTYPE0_RST        0x0
#define BT_SYNLTADDR0_RST      0x0

__INLINE void bt_e_scoltcntl_0_pack(uint8_t retxnb0, uint8_t escoedrrx0, uint8_t escoedrtx0, uint8_t syntype0, uint8_t synltaddr0)
{
    ASSERT_ERR((((uint32_t)retxnb0 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)escoedrrx0 << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)escoedrtx0 << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)syntype0 << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)synltaddr0 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_0_ADDR, ((uint32_t)retxnb0 << 16) | ((uint32_t)escoedrrx0 << 5) | ((uint32_t)escoedrtx0 << 4) | ((uint32_t)syntype0 << 3) | ((uint32_t)synltaddr0 << 0));
}

__INLINE void bt_e_scoltcntl_0_unpack(uint8_t *retxnb0, uint8_t *escoedrrx0, uint8_t *escoedrtx0, uint8_t *syntype0, uint8_t *synltaddr0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR);

    *retxnb0 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *escoedrrx0 = (localVal & ((uint32_t)0x00000020)) >> 5;
    *escoedrtx0 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *syntype0 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *synltaddr0 = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_e_scoltcntl_0_retxnb0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bt_e_scoltcntl_0_retxnb0_setf(uint8_t retxnb0)
{
    ASSERT_ERR((((uint32_t)retxnb0 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)retxnb0 << 16));
}

__INLINE uint8_t bt_e_scoltcntl_0_e_scoedrrx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_e_scoltcntl_0_e_scoedrrx_0_setf(uint8_t escoedrrx0)
{
    ASSERT_ERR((((uint32_t)escoedrrx0 << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)escoedrrx0 << 5));
}

__INLINE uint8_t bt_e_scoltcntl_0_e_scoedrtx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_e_scoltcntl_0_e_scoedrtx_0_setf(uint8_t escoedrtx0)
{
    ASSERT_ERR((((uint32_t)escoedrtx0 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)escoedrtx0 << 4));
}

__INLINE uint8_t bt_e_scoltcntl_0_syntype0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_e_scoltcntl_0_syntype0_setf(uint8_t syntype0)
{
    ASSERT_ERR((((uint32_t)syntype0 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)syntype0 << 3));
}

__INLINE uint8_t bt_e_scoltcntl_0_synltaddr0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_e_scoltcntl_0_synltaddr0_setf(uint8_t synltaddr0)
{
    ASSERT_ERR((((uint32_t)synltaddr0 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_0_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)synltaddr0 << 0));
}

/**
 * @brief E_SCOTRCNTL_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31              TXSEQN0   0
 *  29:20               TXLEN0   0x0
 *  19:16              TXTYPE0   0x0
 *  13:04               RXLEN0   0x0
 *  03:00              RXTYPE0   0x0
 * </pre>
 */
#define BT_E_SCOTRCNTL_0_ADDR   0x50090624
#define BT_E_SCOTRCNTL_0_OFFSET 0x00000224
#define BT_E_SCOTRCNTL_0_INDEX  0x00000089
#define BT_E_SCOTRCNTL_0_RESET  0x00000000

__INLINE uint32_t bt_e_scotrcntl_0_get(void)
{
    return REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR);
}

__INLINE void bt_e_scotrcntl_0_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOTRCNTL_0_ADDR, value);
}

// field definitions
#define BT_TXSEQN0_BIT    ((uint32_t)0x80000000)
#define BT_TXSEQN0_POS    31
#define BT_TXLEN0_MASK    ((uint32_t)0x3FF00000)
#define BT_TXLEN0_LSB     20
#define BT_TXLEN0_WIDTH   ((uint32_t)0x0000000A)
#define BT_TXTYPE0_MASK   ((uint32_t)0x000F0000)
#define BT_TXTYPE0_LSB    16
#define BT_TXTYPE0_WIDTH  ((uint32_t)0x00000004)
#define BT_RXLEN0_MASK    ((uint32_t)0x00003FF0)
#define BT_RXLEN0_LSB     4
#define BT_RXLEN0_WIDTH   ((uint32_t)0x0000000A)
#define BT_RXTYPE0_MASK   ((uint32_t)0x0000000F)
#define BT_RXTYPE0_LSB    0
#define BT_RXTYPE0_WIDTH  ((uint32_t)0x00000004)

#define BT_TXSEQN0_RST    0x0
#define BT_TXLEN0_RST     0x0
#define BT_TXTYPE0_RST    0x0
#define BT_RXLEN0_RST     0x0
#define BT_RXTYPE0_RST    0x0

__INLINE void bt_e_scotrcntl_0_pack(uint8_t txseqn0, uint16_t txlen0, uint8_t txtype0, uint16_t rxlen0, uint8_t rxtype0)
{
    ASSERT_ERR((((uint32_t)txseqn0 << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txlen0 << 20) & ~((uint32_t)0x3FF00000)) == 0);
    ASSERT_ERR((((uint32_t)txtype0 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxlen0 << 4) & ~((uint32_t)0x00003FF0)) == 0);
    ASSERT_ERR((((uint32_t)rxtype0 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_0_ADDR, ((uint32_t)txseqn0 << 31) | ((uint32_t)txlen0 << 20) | ((uint32_t)txtype0 << 16) | ((uint32_t)rxlen0 << 4) | ((uint32_t)rxtype0 << 0));
}

__INLINE void bt_e_scotrcntl_0_unpack(uint8_t *txseqn0, uint16_t *txlen0, uint8_t *txtype0, uint16_t *rxlen0, uint8_t *rxtype0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR);

    *txseqn0 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txlen0 = (localVal & ((uint32_t)0x3FF00000)) >> 20;
    *txtype0 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rxlen0 = (localVal & ((uint32_t)0x00003FF0)) >> 4;
    *rxtype0 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_e_scotrcntl_0_txseqn0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_e_scotrcntl_0_txseqn0_setf(uint8_t txseqn0)
{
    ASSERT_ERR((((uint32_t)txseqn0 << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txseqn0 << 31));
}

__INLINE uint16_t bt_e_scotrcntl_0_txlen0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x3FF00000)) >> 20);
}

__INLINE void bt_e_scotrcntl_0_txlen0_setf(uint16_t txlen0)
{
    ASSERT_ERR((((uint32_t)txlen0 << 20) & ~((uint32_t)0x3FF00000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR) & ~((uint32_t)0x3FF00000)) | ((uint32_t)txlen0 << 20));
}

__INLINE uint8_t bt_e_scotrcntl_0_txtype0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_e_scotrcntl_0_txtype0_setf(uint8_t txtype0)
{
    ASSERT_ERR((((uint32_t)txtype0 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)txtype0 << 16));
}

__INLINE uint16_t bt_e_scotrcntl_0_rxlen0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x00003FF0)) >> 4);
}

__INLINE void bt_e_scotrcntl_0_rxlen0_setf(uint16_t rxlen0)
{
    ASSERT_ERR((((uint32_t)rxlen0 << 4) & ~((uint32_t)0x00003FF0)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR) & ~((uint32_t)0x00003FF0)) | ((uint32_t)rxlen0 << 4));
}

__INLINE uint8_t bt_e_scotrcntl_0_rxtype0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_e_scotrcntl_0_rxtype0_setf(uint8_t rxtype0)
{
    ASSERT_ERR((((uint32_t)rxtype0 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_0_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_0_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)rxtype0 << 0));
}

/**
 * @brief E_SCODAYCNT_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  10:00          DAYCOUNTER0   0x0
 * </pre>
 */
#define BT_E_SCODAYCNT_0_ADDR   0x50090628
#define BT_E_SCODAYCNT_0_OFFSET 0x00000228
#define BT_E_SCODAYCNT_0_INDEX  0x0000008A
#define BT_E_SCODAYCNT_0_RESET  0x00000000

__INLINE uint32_t bt_e_scodaycnt_0_get(void)
{
    return REG_BT_RD(BT_E_SCODAYCNT_0_ADDR);
}

__INLINE void bt_e_scodaycnt_0_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCODAYCNT_0_ADDR, value);
}

// field definitions
#define BT_DAYCOUNTER0_MASK   ((uint32_t)0x000007FF)
#define BT_DAYCOUNTER0_LSB    0
#define BT_DAYCOUNTER0_WIDTH  ((uint32_t)0x0000000B)

#define BT_DAYCOUNTER0_RST    0x0

__INLINE uint16_t bt_e_scodaycnt_0_daycounter0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCODAYCNT_0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000007FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_e_scodaycnt_0_daycounter0_setf(uint16_t daycounter0)
{
    ASSERT_ERR((((uint32_t)daycounter0 << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_E_SCODAYCNT_0_ADDR, (uint32_t)daycounter0 << 0);
}

/**
 * @brief E_SCOCHANCNTL_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                 TOG1   0
 *     15        eSCOCHANSWEN1   0
 *     14          eSCOCHANEN1   0
 *     13              ITMODE1   0
 *  12:08            INTDELAY1   0x0
 *  07:00               TeSCO1   0x0
 * </pre>
 */
#define BT_E_SCOCHANCNTL_1_ADDR   0x50090630
#define BT_E_SCOCHANCNTL_1_OFFSET 0x00000230
#define BT_E_SCOCHANCNTL_1_INDEX  0x0000008C
#define BT_E_SCOCHANCNTL_1_RESET  0x00000000

__INLINE uint32_t bt_e_scochancntl_1_get(void)
{
    return REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);
}

__INLINE void bt_e_scochancntl_1_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCHANCNTL_1_ADDR, value);
}

// field definitions
#define BT_TOG1_BIT               ((uint32_t)0x80000000)
#define BT_TOG1_POS               31
#define BT_E_SCOCHANSWEN_1_BIT    ((uint32_t)0x00008000)
#define BT_E_SCOCHANSWEN_1_POS    15
#define BT_E_SCOCHANEN_1_BIT      ((uint32_t)0x00004000)
#define BT_E_SCOCHANEN_1_POS      14
#define BT_ITMODE1_BIT            ((uint32_t)0x00002000)
#define BT_ITMODE1_POS            13
#define BT_INTDELAY1_MASK         ((uint32_t)0x00001F00)
#define BT_INTDELAY1_LSB          8
#define BT_INTDELAY1_WIDTH        ((uint32_t)0x00000005)
#define BT_TE_SCO_1_MASK          ((uint32_t)0x000000FF)
#define BT_TE_SCO_1_LSB           0
#define BT_TE_SCO_1_WIDTH         ((uint32_t)0x00000008)

#define BT_TOG1_RST               0x0
#define BT_E_SCOCHANSWEN_1_RST    0x0
#define BT_E_SCOCHANEN_1_RST      0x0
#define BT_ITMODE1_RST            0x0
#define BT_INTDELAY1_RST          0x0
#define BT_TE_SCO_1_RST           0x0

__INLINE void bt_e_scochancntl_1_pack(uint8_t escochanswen1, uint8_t escochanen1, uint8_t itmode1, uint8_t intdelay1, uint8_t tesco1)
{
    ASSERT_ERR((((uint32_t)escochanswen1 << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)escochanen1 << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)itmode1 << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)intdelay1 << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)tesco1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_1_ADDR, ((uint32_t)escochanswen1 << 15) | ((uint32_t)escochanen1 << 14) | ((uint32_t)itmode1 << 13) | ((uint32_t)intdelay1 << 8) | ((uint32_t)tesco1 << 0));
}

__INLINE void bt_e_scochancntl_1_unpack(uint8_t *tog1, uint8_t *escochanswen1, uint8_t *escochanen1, uint8_t *itmode1, uint8_t *intdelay1, uint8_t *tesco1)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);

    *tog1 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *escochanswen1 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *escochanen1 = (localVal & ((uint32_t)0x00004000)) >> 14;
    *itmode1 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *intdelay1 = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *tesco1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bt_e_scochancntl_1_tog1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t bt_e_scochancntl_1_e_scochanswen_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_e_scochancntl_1_e_scochanswen_1_setf(uint8_t escochanswen1)
{
    ASSERT_ERR((((uint32_t)escochanswen1 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)escochanswen1 << 15));
}

__INLINE uint8_t bt_e_scochancntl_1_e_scochanen_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_e_scochancntl_1_e_scochanen_1_setf(uint8_t escochanen1)
{
    ASSERT_ERR((((uint32_t)escochanen1 << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)escochanen1 << 14));
}

__INLINE uint8_t bt_e_scochancntl_1_itmode1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_e_scochancntl_1_itmode1_setf(uint8_t itmode1)
{
    ASSERT_ERR((((uint32_t)itmode1 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)itmode1 << 13));
}

__INLINE uint8_t bt_e_scochancntl_1_intdelay1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void bt_e_scochancntl_1_intdelay1_setf(uint8_t intdelay1)
{
    ASSERT_ERR((((uint32_t)intdelay1 << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)intdelay1 << 8));
}

__INLINE uint8_t bt_e_scochancntl_1_te_sco_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bt_e_scochancntl_1_te_sco_1_setf(uint8_t tesco1)
{
    ASSERT_ERR((((uint32_t)tesco1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_1_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)tesco1 << 0));
}

/**
 * @brief E_SCOMUTECNTL_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     23           MUTE_SINK1   0
 *     22         MUTE_SOURCE1   0
 *  19:18              INVL1_1   0x2
 *  17:16              INVL1_0   0x2
 *  15:00            MUTEPATT1   0x0
 * </pre>
 */
#define BT_E_SCOMUTECNTL_1_ADDR   0x50090634
#define BT_E_SCOMUTECNTL_1_OFFSET 0x00000234
#define BT_E_SCOMUTECNTL_1_INDEX  0x0000008D
#define BT_E_SCOMUTECNTL_1_RESET  0x000A0000

__INLINE uint32_t bt_e_scomutecntl_1_get(void)
{
    return REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR);
}

__INLINE void bt_e_scomutecntl_1_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOMUTECNTL_1_ADDR, value);
}

// field definitions
#define BT_MUTE_SINK1_BIT      ((uint32_t)0x00800000)
#define BT_MUTE_SINK1_POS      23
#define BT_MUTE_SOURCE1_BIT    ((uint32_t)0x00400000)
#define BT_MUTE_SOURCE1_POS    22
#define BT_INVL1_1_MASK        ((uint32_t)0x000C0000)
#define BT_INVL1_1_LSB         18
#define BT_INVL1_1_WIDTH       ((uint32_t)0x00000002)
#define BT_INVL1_0_MASK        ((uint32_t)0x00030000)
#define BT_INVL1_0_LSB         16
#define BT_INVL1_0_WIDTH       ((uint32_t)0x00000002)
#define BT_MUTEPATT1_MASK      ((uint32_t)0x0000FFFF)
#define BT_MUTEPATT1_LSB       0
#define BT_MUTEPATT1_WIDTH     ((uint32_t)0x00000010)

#define BT_MUTE_SINK1_RST      0x0
#define BT_MUTE_SOURCE1_RST    0x0
#define BT_INVL1_1_RST         0x2
#define BT_INVL1_0_RST         0x2
#define BT_MUTEPATT1_RST       0x0

__INLINE void bt_e_scomutecntl_1_pack(uint8_t mutesink1, uint8_t mutesource1, uint8_t invl11, uint8_t invl10, uint16_t mutepatt1)
{
    ASSERT_ERR((((uint32_t)mutesink1 << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)mutesource1 << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)invl11 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)invl10 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mutepatt1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_1_ADDR, ((uint32_t)mutesink1 << 23) | ((uint32_t)mutesource1 << 22) | ((uint32_t)invl11 << 18) | ((uint32_t)invl10 << 16) | ((uint32_t)mutepatt1 << 0));
}

__INLINE void bt_e_scomutecntl_1_unpack(uint8_t *mutesink1, uint8_t *mutesource1, uint8_t *invl11, uint8_t *invl10, uint16_t *mutepatt1)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR);

    *mutesink1 = (localVal & ((uint32_t)0x00800000)) >> 23;
    *mutesource1 = (localVal & ((uint32_t)0x00400000)) >> 22;
    *invl11 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *invl10 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mutepatt1 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t bt_e_scomutecntl_1_mute_sink1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bt_e_scomutecntl_1_mute_sink1_setf(uint8_t mutesink1)
{
    ASSERT_ERR((((uint32_t)mutesink1 << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_1_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)mutesink1 << 23));
}

__INLINE uint8_t bt_e_scomutecntl_1_mute_source1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bt_e_scomutecntl_1_mute_source1_setf(uint8_t mutesource1)
{
    ASSERT_ERR((((uint32_t)mutesource1 << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_1_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)mutesource1 << 22));
}

__INLINE uint8_t bt_e_scomutecntl_1_invl1_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void bt_e_scomutecntl_1_invl1_1_setf(uint8_t invl11)
{
    ASSERT_ERR((((uint32_t)invl11 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_1_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)invl11 << 18));
}

__INLINE uint8_t bt_e_scomutecntl_1_invl1_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_e_scomutecntl_1_invl1_0_setf(uint8_t invl10)
{
    ASSERT_ERR((((uint32_t)invl10 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_1_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)invl10 << 16));
}

__INLINE uint16_t bt_e_scomutecntl_1_mutepatt1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_e_scomutecntl_1_mutepatt1_setf(uint16_t mutepatt1)
{
    ASSERT_ERR((((uint32_t)mutepatt1 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_1_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_1_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mutepatt1 << 0));
}

/**
 * @brief E_SCOCURRENTTXPTR_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          eSCO1PTRTX1   0x0
 *  13:00          eSCO1PTRTX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTTXPTR_1_ADDR   0x50090638
#define BT_E_SCOCURRENTTXPTR_1_OFFSET 0x00000238
#define BT_E_SCOCURRENTTXPTR_1_INDEX  0x0000008E
#define BT_E_SCOCURRENTTXPTR_1_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrenttxptr_1_get(void)
{
    return REG_BT_RD(BT_E_SCOCURRENTTXPTR_1_ADDR);
}

__INLINE void bt_e_scocurrenttxptr_1_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_1_ADDR, value);
}

// field definitions
#define BT_E_SCO_1PTRTX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCO_1PTRTX_1_LSB    16
#define BT_E_SCO_1PTRTX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCO_1PTRTX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCO_1PTRTX_0_LSB    0
#define BT_E_SCO_1PTRTX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCO_1PTRTX_1_RST    0x0
#define BT_E_SCO_1PTRTX_0_RST    0x0

__INLINE void bt_e_scocurrenttxptr_1_pack(uint16_t esco1ptrtx1, uint16_t esco1ptrtx0)
{
    ASSERT_ERR((((uint32_t)esco1ptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)esco1ptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_1_ADDR, ((uint32_t)esco1ptrtx1 << 16) | ((uint32_t)esco1ptrtx0 << 0));
}

__INLINE void bt_e_scocurrenttxptr_1_unpack(uint16_t *esco1ptrtx1, uint16_t *esco1ptrtx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_1_ADDR);

    *esco1ptrtx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *esco1ptrtx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrenttxptr_1_e_sco_1ptrtx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_1_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrenttxptr_1_e_sco_1ptrtx_1_setf(uint16_t esco1ptrtx1)
{
    ASSERT_ERR((((uint32_t)esco1ptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_1_ADDR, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_1_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)esco1ptrtx1 << 16));
}

__INLINE uint16_t bt_e_scocurrenttxptr_1_e_sco_1ptrtx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_1_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrenttxptr_1_e_sco_1ptrtx_0_setf(uint16_t esco1ptrtx0)
{
    ASSERT_ERR((((uint32_t)esco1ptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_1_ADDR, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_1_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)esco1ptrtx0 << 0));
}

/**
 * @brief E_SCOCURRENTRXPTR_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          eSCO1PTRRX1   0x0
 *  13:00          eSCO1PTRRX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTRXPTR_1_ADDR   0x5009063C
#define BT_E_SCOCURRENTRXPTR_1_OFFSET 0x0000023C
#define BT_E_SCOCURRENTRXPTR_1_INDEX  0x0000008F
#define BT_E_SCOCURRENTRXPTR_1_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrentrxptr_1_get(void)
{
    return REG_BT_RD(BT_E_SCOCURRENTRXPTR_1_ADDR);
}

__INLINE void bt_e_scocurrentrxptr_1_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_1_ADDR, value);
}

// field definitions
#define BT_E_SCO_1PTRRX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCO_1PTRRX_1_LSB    16
#define BT_E_SCO_1PTRRX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCO_1PTRRX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCO_1PTRRX_0_LSB    0
#define BT_E_SCO_1PTRRX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCO_1PTRRX_1_RST    0x0
#define BT_E_SCO_1PTRRX_0_RST    0x0

__INLINE void bt_e_scocurrentrxptr_1_pack(uint16_t esco1ptrrx1, uint16_t esco1ptrrx0)
{
    ASSERT_ERR((((uint32_t)esco1ptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)esco1ptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_1_ADDR, ((uint32_t)esco1ptrrx1 << 16) | ((uint32_t)esco1ptrrx0 << 0));
}

__INLINE void bt_e_scocurrentrxptr_1_unpack(uint16_t *esco1ptrrx1, uint16_t *esco1ptrrx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_1_ADDR);

    *esco1ptrrx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *esco1ptrrx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrentrxptr_1_e_sco_1ptrrx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_1_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrentrxptr_1_e_sco_1ptrrx_1_setf(uint16_t esco1ptrrx1)
{
    ASSERT_ERR((((uint32_t)esco1ptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_1_ADDR, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_1_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)esco1ptrrx1 << 16));
}

__INLINE uint16_t bt_e_scocurrentrxptr_1_e_sco_1ptrrx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_1_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrentrxptr_1_e_sco_1ptrrx_0_setf(uint16_t esco1ptrrx0)
{
    ASSERT_ERR((((uint32_t)esco1ptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_1_ADDR, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_1_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)esco1ptrrx0 << 0));
}

/**
 * @brief E_SCOLTCNTL_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16              RETXNB1   0x1
 *     05           eSCOEDRRX1   0
 *     04           eSCOEDRTX1   0
 *     03             SYNTYPE1   0
 *  02:00           SYNLTADDR1   0x0
 * </pre>
 */
#define BT_E_SCOLTCNTL_1_ADDR   0x50090640
#define BT_E_SCOLTCNTL_1_OFFSET 0x00000240
#define BT_E_SCOLTCNTL_1_INDEX  0x00000090
#define BT_E_SCOLTCNTL_1_RESET  0x00010000

__INLINE uint32_t bt_e_scoltcntl_1_get(void)
{
    return REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR);
}

__INLINE void bt_e_scoltcntl_1_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOLTCNTL_1_ADDR, value);
}

// field definitions
#define BT_RETXNB1_MASK        ((uint32_t)0x00FF0000)
#define BT_RETXNB1_LSB         16
#define BT_RETXNB1_WIDTH       ((uint32_t)0x00000008)
#define BT_E_SCOEDRRX_1_BIT    ((uint32_t)0x00000020)
#define BT_E_SCOEDRRX_1_POS    5
#define BT_E_SCOEDRTX_1_BIT    ((uint32_t)0x00000010)
#define BT_E_SCOEDRTX_1_POS    4
#define BT_SYNTYPE1_BIT        ((uint32_t)0x00000008)
#define BT_SYNTYPE1_POS        3
#define BT_SYNLTADDR1_MASK     ((uint32_t)0x00000007)
#define BT_SYNLTADDR1_LSB      0
#define BT_SYNLTADDR1_WIDTH    ((uint32_t)0x00000003)

#define BT_RETXNB1_RST         0x1
#define BT_E_SCOEDRRX_1_RST    0x0
#define BT_E_SCOEDRTX_1_RST    0x0
#define BT_SYNTYPE1_RST        0x0
#define BT_SYNLTADDR1_RST      0x0

__INLINE void bt_e_scoltcntl_1_pack(uint8_t retxnb1, uint8_t escoedrrx1, uint8_t escoedrtx1, uint8_t syntype1, uint8_t synltaddr1)
{
    ASSERT_ERR((((uint32_t)retxnb1 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)escoedrrx1 << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)escoedrtx1 << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)syntype1 << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)synltaddr1 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_1_ADDR, ((uint32_t)retxnb1 << 16) | ((uint32_t)escoedrrx1 << 5) | ((uint32_t)escoedrtx1 << 4) | ((uint32_t)syntype1 << 3) | ((uint32_t)synltaddr1 << 0));
}

__INLINE void bt_e_scoltcntl_1_unpack(uint8_t *retxnb1, uint8_t *escoedrrx1, uint8_t *escoedrtx1, uint8_t *syntype1, uint8_t *synltaddr1)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR);

    *retxnb1 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *escoedrrx1 = (localVal & ((uint32_t)0x00000020)) >> 5;
    *escoedrtx1 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *syntype1 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *synltaddr1 = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_e_scoltcntl_1_retxnb1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bt_e_scoltcntl_1_retxnb1_setf(uint8_t retxnb1)
{
    ASSERT_ERR((((uint32_t)retxnb1 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)retxnb1 << 16));
}

__INLINE uint8_t bt_e_scoltcntl_1_e_scoedrrx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_e_scoltcntl_1_e_scoedrrx_1_setf(uint8_t escoedrrx1)
{
    ASSERT_ERR((((uint32_t)escoedrrx1 << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)escoedrrx1 << 5));
}

__INLINE uint8_t bt_e_scoltcntl_1_e_scoedrtx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_e_scoltcntl_1_e_scoedrtx_1_setf(uint8_t escoedrtx1)
{
    ASSERT_ERR((((uint32_t)escoedrtx1 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)escoedrtx1 << 4));
}

__INLINE uint8_t bt_e_scoltcntl_1_syntype1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_e_scoltcntl_1_syntype1_setf(uint8_t syntype1)
{
    ASSERT_ERR((((uint32_t)syntype1 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)syntype1 << 3));
}

__INLINE uint8_t bt_e_scoltcntl_1_synltaddr1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_e_scoltcntl_1_synltaddr1_setf(uint8_t synltaddr1)
{
    ASSERT_ERR((((uint32_t)synltaddr1 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_1_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)synltaddr1 << 0));
}

/**
 * @brief E_SCOTRCNTL_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31              TXSEQN1   0
 *  29:20               TXLEN1   0x0
 *  19:16              TXTYPE1   0x0
 *  13:04               RXLEN1   0x0
 *  03:00              RXTYPE1   0x0
 * </pre>
 */
#define BT_E_SCOTRCNTL_1_ADDR   0x50090644
#define BT_E_SCOTRCNTL_1_OFFSET 0x00000244
#define BT_E_SCOTRCNTL_1_INDEX  0x00000091
#define BT_E_SCOTRCNTL_1_RESET  0x00000000

__INLINE uint32_t bt_e_scotrcntl_1_get(void)
{
    return REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR);
}

__INLINE void bt_e_scotrcntl_1_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOTRCNTL_1_ADDR, value);
}

// field definitions
#define BT_TXSEQN1_BIT    ((uint32_t)0x80000000)
#define BT_TXSEQN1_POS    31
#define BT_TXLEN1_MASK    ((uint32_t)0x3FF00000)
#define BT_TXLEN1_LSB     20
#define BT_TXLEN1_WIDTH   ((uint32_t)0x0000000A)
#define BT_TXTYPE1_MASK   ((uint32_t)0x000F0000)
#define BT_TXTYPE1_LSB    16
#define BT_TXTYPE1_WIDTH  ((uint32_t)0x00000004)
#define BT_RXLEN1_MASK    ((uint32_t)0x00003FF0)
#define BT_RXLEN1_LSB     4
#define BT_RXLEN1_WIDTH   ((uint32_t)0x0000000A)
#define BT_RXTYPE1_MASK   ((uint32_t)0x0000000F)
#define BT_RXTYPE1_LSB    0
#define BT_RXTYPE1_WIDTH  ((uint32_t)0x00000004)

#define BT_TXSEQN1_RST    0x0
#define BT_TXLEN1_RST     0x0
#define BT_TXTYPE1_RST    0x0
#define BT_RXLEN1_RST     0x0
#define BT_RXTYPE1_RST    0x0

__INLINE void bt_e_scotrcntl_1_pack(uint8_t txseqn1, uint16_t txlen1, uint8_t txtype1, uint16_t rxlen1, uint8_t rxtype1)
{
    ASSERT_ERR((((uint32_t)txseqn1 << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txlen1 << 20) & ~((uint32_t)0x3FF00000)) == 0);
    ASSERT_ERR((((uint32_t)txtype1 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxlen1 << 4) & ~((uint32_t)0x00003FF0)) == 0);
    ASSERT_ERR((((uint32_t)rxtype1 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_1_ADDR, ((uint32_t)txseqn1 << 31) | ((uint32_t)txlen1 << 20) | ((uint32_t)txtype1 << 16) | ((uint32_t)rxlen1 << 4) | ((uint32_t)rxtype1 << 0));
}

__INLINE void bt_e_scotrcntl_1_unpack(uint8_t *txseqn1, uint16_t *txlen1, uint8_t *txtype1, uint16_t *rxlen1, uint8_t *rxtype1)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR);

    *txseqn1 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txlen1 = (localVal & ((uint32_t)0x3FF00000)) >> 20;
    *txtype1 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rxlen1 = (localVal & ((uint32_t)0x00003FF0)) >> 4;
    *rxtype1 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_e_scotrcntl_1_txseqn1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_e_scotrcntl_1_txseqn1_setf(uint8_t txseqn1)
{
    ASSERT_ERR((((uint32_t)txseqn1 << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txseqn1 << 31));
}

__INLINE uint16_t bt_e_scotrcntl_1_txlen1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x3FF00000)) >> 20);
}

__INLINE void bt_e_scotrcntl_1_txlen1_setf(uint16_t txlen1)
{
    ASSERT_ERR((((uint32_t)txlen1 << 20) & ~((uint32_t)0x3FF00000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR) & ~((uint32_t)0x3FF00000)) | ((uint32_t)txlen1 << 20));
}

__INLINE uint8_t bt_e_scotrcntl_1_txtype1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_e_scotrcntl_1_txtype1_setf(uint8_t txtype1)
{
    ASSERT_ERR((((uint32_t)txtype1 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)txtype1 << 16));
}

__INLINE uint16_t bt_e_scotrcntl_1_rxlen1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x00003FF0)) >> 4);
}

__INLINE void bt_e_scotrcntl_1_rxlen1_setf(uint16_t rxlen1)
{
    ASSERT_ERR((((uint32_t)rxlen1 << 4) & ~((uint32_t)0x00003FF0)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR) & ~((uint32_t)0x00003FF0)) | ((uint32_t)rxlen1 << 4));
}

__INLINE uint8_t bt_e_scotrcntl_1_rxtype1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_e_scotrcntl_1_rxtype1_setf(uint8_t rxtype1)
{
    ASSERT_ERR((((uint32_t)rxtype1 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_1_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)rxtype1 << 0));
}

/**
 * @brief E_SCODAYCNT_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  10:00          DAYCOUNTER1   0x0
 * </pre>
 */
#define BT_E_SCODAYCNT_1_ADDR   0x50090648
#define BT_E_SCODAYCNT_1_OFFSET 0x00000248
#define BT_E_SCODAYCNT_1_INDEX  0x00000092
#define BT_E_SCODAYCNT_1_RESET  0x00000000

__INLINE uint32_t bt_e_scodaycnt_1_get(void)
{
    return REG_BT_RD(BT_E_SCODAYCNT_1_ADDR);
}

__INLINE void bt_e_scodaycnt_1_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCODAYCNT_1_ADDR, value);
}

// field definitions
#define BT_DAYCOUNTER1_MASK   ((uint32_t)0x000007FF)
#define BT_DAYCOUNTER1_LSB    0
#define BT_DAYCOUNTER1_WIDTH  ((uint32_t)0x0000000B)

#define BT_DAYCOUNTER1_RST    0x0

__INLINE uint16_t bt_e_scodaycnt_1_daycounter1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCODAYCNT_1_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000007FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_e_scodaycnt_1_daycounter1_setf(uint16_t daycounter1)
{
    ASSERT_ERR((((uint32_t)daycounter1 << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_E_SCODAYCNT_1_ADDR, (uint32_t)daycounter1 << 0);
}

/**
 * @brief E_SCOCHANCNTL_2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                 TOG2   0
 *     15        eSCOCHANSWEN2   0
 *     14          eSCOCHANEN2   0
 *     13              ITMODE2   0
 *  12:08            INTDELAY2   0x0
 *  07:00               TeSCO2   0x0
 * </pre>
 */
#define BT_E_SCOCHANCNTL_2_ADDR   0x50090650
#define BT_E_SCOCHANCNTL_2_OFFSET 0x00000250
#define BT_E_SCOCHANCNTL_2_INDEX  0x00000094
#define BT_E_SCOCHANCNTL_2_RESET  0x00000000

__INLINE uint32_t bt_e_scochancntl_2_get(void)
{
    return REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);
}

__INLINE void bt_e_scochancntl_2_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCHANCNTL_2_ADDR, value);
}

// field definitions
#define BT_TOG2_BIT               ((uint32_t)0x80000000)
#define BT_TOG2_POS               31
#define BT_E_SCOCHANSWEN_2_BIT    ((uint32_t)0x00008000)
#define BT_E_SCOCHANSWEN_2_POS    15
#define BT_E_SCOCHANEN_2_BIT      ((uint32_t)0x00004000)
#define BT_E_SCOCHANEN_2_POS      14
#define BT_ITMODE2_BIT            ((uint32_t)0x00002000)
#define BT_ITMODE2_POS            13
#define BT_INTDELAY2_MASK         ((uint32_t)0x00001F00)
#define BT_INTDELAY2_LSB          8
#define BT_INTDELAY2_WIDTH        ((uint32_t)0x00000005)
#define BT_TE_SCO_2_MASK          ((uint32_t)0x000000FF)
#define BT_TE_SCO_2_LSB           0
#define BT_TE_SCO_2_WIDTH         ((uint32_t)0x00000008)

#define BT_TOG2_RST               0x0
#define BT_E_SCOCHANSWEN_2_RST    0x0
#define BT_E_SCOCHANEN_2_RST      0x0
#define BT_ITMODE2_RST            0x0
#define BT_INTDELAY2_RST          0x0
#define BT_TE_SCO_2_RST           0x0

__INLINE void bt_e_scochancntl_2_pack(uint8_t escochanswen2, uint8_t escochanen2, uint8_t itmode2, uint8_t intdelay2, uint8_t tesco2)
{
    ASSERT_ERR((((uint32_t)escochanswen2 << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)escochanen2 << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)itmode2 << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)intdelay2 << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)tesco2 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_2_ADDR, ((uint32_t)escochanswen2 << 15) | ((uint32_t)escochanen2 << 14) | ((uint32_t)itmode2 << 13) | ((uint32_t)intdelay2 << 8) | ((uint32_t)tesco2 << 0));
}

__INLINE void bt_e_scochancntl_2_unpack(uint8_t *tog2, uint8_t *escochanswen2, uint8_t *escochanen2, uint8_t *itmode2, uint8_t *intdelay2, uint8_t *tesco2)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);

    *tog2 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *escochanswen2 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *escochanen2 = (localVal & ((uint32_t)0x00004000)) >> 14;
    *itmode2 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *intdelay2 = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *tesco2 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bt_e_scochancntl_2_tog2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t bt_e_scochancntl_2_e_scochanswen_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_e_scochancntl_2_e_scochanswen_2_setf(uint8_t escochanswen2)
{
    ASSERT_ERR((((uint32_t)escochanswen2 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)escochanswen2 << 15));
}

__INLINE uint8_t bt_e_scochancntl_2_e_scochanen_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bt_e_scochancntl_2_e_scochanen_2_setf(uint8_t escochanen2)
{
    ASSERT_ERR((((uint32_t)escochanen2 << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)escochanen2 << 14));
}

__INLINE uint8_t bt_e_scochancntl_2_itmode2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_e_scochancntl_2_itmode2_setf(uint8_t itmode2)
{
    ASSERT_ERR((((uint32_t)itmode2 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)itmode2 << 13));
}

__INLINE uint8_t bt_e_scochancntl_2_intdelay2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void bt_e_scochancntl_2_intdelay2_setf(uint8_t intdelay2)
{
    ASSERT_ERR((((uint32_t)intdelay2 << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)intdelay2 << 8));
}

__INLINE uint8_t bt_e_scochancntl_2_te_sco_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bt_e_scochancntl_2_te_sco_2_setf(uint8_t tesco2)
{
    ASSERT_ERR((((uint32_t)tesco2 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(BT_E_SCOCHANCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOCHANCNTL_2_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)tesco2 << 0));
}

/**
 * @brief E_SCOMUTECNTL_2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     23           MUTE_SINK2   0
 *     22         MUTE_SOURCE2   0
 *  19:18              INVL2_1   0x2
 *  17:16              INVL2_0   0x2
 *  15:00            MUTEPATT2   0x0
 * </pre>
 */
#define BT_E_SCOMUTECNTL_2_ADDR   0x50090654
#define BT_E_SCOMUTECNTL_2_OFFSET 0x00000254
#define BT_E_SCOMUTECNTL_2_INDEX  0x00000095
#define BT_E_SCOMUTECNTL_2_RESET  0x000A0000

__INLINE uint32_t bt_e_scomutecntl_2_get(void)
{
    return REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR);
}

__INLINE void bt_e_scomutecntl_2_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOMUTECNTL_2_ADDR, value);
}

// field definitions
#define BT_MUTE_SINK2_BIT      ((uint32_t)0x00800000)
#define BT_MUTE_SINK2_POS      23
#define BT_MUTE_SOURCE2_BIT    ((uint32_t)0x00400000)
#define BT_MUTE_SOURCE2_POS    22
#define BT_INVL2_1_MASK        ((uint32_t)0x000C0000)
#define BT_INVL2_1_LSB         18
#define BT_INVL2_1_WIDTH       ((uint32_t)0x00000002)
#define BT_INVL2_0_MASK        ((uint32_t)0x00030000)
#define BT_INVL2_0_LSB         16
#define BT_INVL2_0_WIDTH       ((uint32_t)0x00000002)
#define BT_MUTEPATT2_MASK      ((uint32_t)0x0000FFFF)
#define BT_MUTEPATT2_LSB       0
#define BT_MUTEPATT2_WIDTH     ((uint32_t)0x00000010)

#define BT_MUTE_SINK2_RST      0x0
#define BT_MUTE_SOURCE2_RST    0x0
#define BT_INVL2_1_RST         0x2
#define BT_INVL2_0_RST         0x2
#define BT_MUTEPATT2_RST       0x0

__INLINE void bt_e_scomutecntl_2_pack(uint8_t mutesink2, uint8_t mutesource2, uint8_t invl21, uint8_t invl20, uint16_t mutepatt2)
{
    ASSERT_ERR((((uint32_t)mutesink2 << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)mutesource2 << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)invl21 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)invl20 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mutepatt2 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_2_ADDR, ((uint32_t)mutesink2 << 23) | ((uint32_t)mutesource2 << 22) | ((uint32_t)invl21 << 18) | ((uint32_t)invl20 << 16) | ((uint32_t)mutepatt2 << 0));
}

__INLINE void bt_e_scomutecntl_2_unpack(uint8_t *mutesink2, uint8_t *mutesource2, uint8_t *invl21, uint8_t *invl20, uint16_t *mutepatt2)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR);

    *mutesink2 = (localVal & ((uint32_t)0x00800000)) >> 23;
    *mutesource2 = (localVal & ((uint32_t)0x00400000)) >> 22;
    *invl21 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *invl20 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mutepatt2 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t bt_e_scomutecntl_2_mute_sink2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bt_e_scomutecntl_2_mute_sink2_setf(uint8_t mutesink2)
{
    ASSERT_ERR((((uint32_t)mutesink2 << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_2_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)mutesink2 << 23));
}

__INLINE uint8_t bt_e_scomutecntl_2_mute_source2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bt_e_scomutecntl_2_mute_source2_setf(uint8_t mutesource2)
{
    ASSERT_ERR((((uint32_t)mutesource2 << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_2_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)mutesource2 << 22));
}

__INLINE uint8_t bt_e_scomutecntl_2_invl2_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void bt_e_scomutecntl_2_invl2_1_setf(uint8_t invl21)
{
    ASSERT_ERR((((uint32_t)invl21 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_2_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)invl21 << 18));
}

__INLINE uint8_t bt_e_scomutecntl_2_invl2_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_e_scomutecntl_2_invl2_0_setf(uint8_t invl20)
{
    ASSERT_ERR((((uint32_t)invl20 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_2_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)invl20 << 16));
}

__INLINE uint16_t bt_e_scomutecntl_2_mutepatt2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_e_scomutecntl_2_mutepatt2_setf(uint16_t mutepatt2)
{
    ASSERT_ERR((((uint32_t)mutepatt2 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_E_SCOMUTECNTL_2_ADDR, (REG_BT_RD(BT_E_SCOMUTECNTL_2_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mutepatt2 << 0));
}

/**
 * @brief E_SCOCURRENTTXPTR_2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          eSCO2PTRTX1   0x0
 *  13:00          eSCO2PTRTX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTTXPTR_2_ADDR   0x50090658
#define BT_E_SCOCURRENTTXPTR_2_OFFSET 0x00000258
#define BT_E_SCOCURRENTTXPTR_2_INDEX  0x00000096
#define BT_E_SCOCURRENTTXPTR_2_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrenttxptr_2_get(void)
{
    return REG_BT_RD(BT_E_SCOCURRENTTXPTR_2_ADDR);
}

__INLINE void bt_e_scocurrenttxptr_2_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_2_ADDR, value);
}

// field definitions
#define BT_E_SCO_2PTRTX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCO_2PTRTX_1_LSB    16
#define BT_E_SCO_2PTRTX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCO_2PTRTX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCO_2PTRTX_0_LSB    0
#define BT_E_SCO_2PTRTX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCO_2PTRTX_1_RST    0x0
#define BT_E_SCO_2PTRTX_0_RST    0x0

__INLINE void bt_e_scocurrenttxptr_2_pack(uint16_t esco2ptrtx1, uint16_t esco2ptrtx0)
{
    ASSERT_ERR((((uint32_t)esco2ptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)esco2ptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_2_ADDR, ((uint32_t)esco2ptrtx1 << 16) | ((uint32_t)esco2ptrtx0 << 0));
}

__INLINE void bt_e_scocurrenttxptr_2_unpack(uint16_t *esco2ptrtx1, uint16_t *esco2ptrtx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_2_ADDR);

    *esco2ptrtx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *esco2ptrtx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrenttxptr_2_e_sco_2ptrtx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_2_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrenttxptr_2_e_sco_2ptrtx_1_setf(uint16_t esco2ptrtx1)
{
    ASSERT_ERR((((uint32_t)esco2ptrtx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_2_ADDR, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_2_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)esco2ptrtx1 << 16));
}

__INLINE uint16_t bt_e_scocurrenttxptr_2_e_sco_2ptrtx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTTXPTR_2_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrenttxptr_2_e_sco_2ptrtx_0_setf(uint16_t esco2ptrtx0)
{
    ASSERT_ERR((((uint32_t)esco2ptrtx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTTXPTR_2_ADDR, (REG_BT_RD(BT_E_SCOCURRENTTXPTR_2_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)esco2ptrtx0 << 0));
}

/**
 * @brief E_SCOCURRENTRXPTR_2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          eSCO2PTRRX1   0x0
 *  13:00          eSCO2PTRRX0   0x0
 * </pre>
 */
#define BT_E_SCOCURRENTRXPTR_2_ADDR   0x5009065C
#define BT_E_SCOCURRENTRXPTR_2_OFFSET 0x0000025C
#define BT_E_SCOCURRENTRXPTR_2_INDEX  0x00000097
#define BT_E_SCOCURRENTRXPTR_2_RESET  0x00000000

__INLINE uint32_t bt_e_scocurrentrxptr_2_get(void)
{
    return REG_BT_RD(BT_E_SCOCURRENTRXPTR_2_ADDR);
}

__INLINE void bt_e_scocurrentrxptr_2_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_2_ADDR, value);
}

// field definitions
#define BT_E_SCO_2PTRRX_1_MASK   ((uint32_t)0x3FFF0000)
#define BT_E_SCO_2PTRRX_1_LSB    16
#define BT_E_SCO_2PTRRX_1_WIDTH  ((uint32_t)0x0000000E)
#define BT_E_SCO_2PTRRX_0_MASK   ((uint32_t)0x00003FFF)
#define BT_E_SCO_2PTRRX_0_LSB    0
#define BT_E_SCO_2PTRRX_0_WIDTH  ((uint32_t)0x0000000E)

#define BT_E_SCO_2PTRRX_1_RST    0x0
#define BT_E_SCO_2PTRRX_0_RST    0x0

__INLINE void bt_e_scocurrentrxptr_2_pack(uint16_t esco2ptrrx1, uint16_t esco2ptrrx0)
{
    ASSERT_ERR((((uint32_t)esco2ptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)esco2ptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_2_ADDR, ((uint32_t)esco2ptrrx1 << 16) | ((uint32_t)esco2ptrrx0 << 0));
}

__INLINE void bt_e_scocurrentrxptr_2_unpack(uint16_t *esco2ptrrx1, uint16_t *esco2ptrrx0)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_2_ADDR);

    *esco2ptrrx1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *esco2ptrrx0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_e_scocurrentrxptr_2_e_sco_2ptrrx_1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_2_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_e_scocurrentrxptr_2_e_sco_2ptrrx_1_setf(uint16_t esco2ptrrx1)
{
    ASSERT_ERR((((uint32_t)esco2ptrrx1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_2_ADDR, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_2_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)esco2ptrrx1 << 16));
}

__INLINE uint16_t bt_e_scocurrentrxptr_2_e_sco_2ptrrx_0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOCURRENTRXPTR_2_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_e_scocurrentrxptr_2_e_sco_2ptrrx_0_setf(uint16_t esco2ptrrx0)
{
    ASSERT_ERR((((uint32_t)esco2ptrrx0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_E_SCOCURRENTRXPTR_2_ADDR, (REG_BT_RD(BT_E_SCOCURRENTRXPTR_2_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)esco2ptrrx0 << 0));
}

/**
 * @brief E_SCOLTCNTL_2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16              RETXNB2   0x1
 *     05           eSCOEDRRX2   0
 *     04           eSCOEDRTX2   0
 *     03             SYNTYPE2   0
 *  02:00           SYNLTADDR2   0x0
 * </pre>
 */
#define BT_E_SCOLTCNTL_2_ADDR   0x50090660
#define BT_E_SCOLTCNTL_2_OFFSET 0x00000260
#define BT_E_SCOLTCNTL_2_INDEX  0x00000098
#define BT_E_SCOLTCNTL_2_RESET  0x00010000

__INLINE uint32_t bt_e_scoltcntl_2_get(void)
{
    return REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR);
}

__INLINE void bt_e_scoltcntl_2_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOLTCNTL_2_ADDR, value);
}

// field definitions
#define BT_RETXNB2_MASK        ((uint32_t)0x00FF0000)
#define BT_RETXNB2_LSB         16
#define BT_RETXNB2_WIDTH       ((uint32_t)0x00000008)
#define BT_E_SCOEDRRX_2_BIT    ((uint32_t)0x00000020)
#define BT_E_SCOEDRRX_2_POS    5
#define BT_E_SCOEDRTX_2_BIT    ((uint32_t)0x00000010)
#define BT_E_SCOEDRTX_2_POS    4
#define BT_SYNTYPE2_BIT        ((uint32_t)0x00000008)
#define BT_SYNTYPE2_POS        3
#define BT_SYNLTADDR2_MASK     ((uint32_t)0x00000007)
#define BT_SYNLTADDR2_LSB      0
#define BT_SYNLTADDR2_WIDTH    ((uint32_t)0x00000003)

#define BT_RETXNB2_RST         0x1
#define BT_E_SCOEDRRX_2_RST    0x0
#define BT_E_SCOEDRTX_2_RST    0x0
#define BT_SYNTYPE2_RST        0x0
#define BT_SYNLTADDR2_RST      0x0

__INLINE void bt_e_scoltcntl_2_pack(uint8_t retxnb2, uint8_t escoedrrx2, uint8_t escoedrtx2, uint8_t syntype2, uint8_t synltaddr2)
{
    ASSERT_ERR((((uint32_t)retxnb2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)escoedrrx2 << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)escoedrtx2 << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)syntype2 << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)synltaddr2 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_2_ADDR, ((uint32_t)retxnb2 << 16) | ((uint32_t)escoedrrx2 << 5) | ((uint32_t)escoedrtx2 << 4) | ((uint32_t)syntype2 << 3) | ((uint32_t)synltaddr2 << 0));
}

__INLINE void bt_e_scoltcntl_2_unpack(uint8_t *retxnb2, uint8_t *escoedrrx2, uint8_t *escoedrtx2, uint8_t *syntype2, uint8_t *synltaddr2)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR);

    *retxnb2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *escoedrrx2 = (localVal & ((uint32_t)0x00000020)) >> 5;
    *escoedrtx2 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *syntype2 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *synltaddr2 = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_e_scoltcntl_2_retxnb2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bt_e_scoltcntl_2_retxnb2_setf(uint8_t retxnb2)
{
    ASSERT_ERR((((uint32_t)retxnb2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)retxnb2 << 16));
}

__INLINE uint8_t bt_e_scoltcntl_2_e_scoedrrx_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_e_scoltcntl_2_e_scoedrrx_2_setf(uint8_t escoedrrx2)
{
    ASSERT_ERR((((uint32_t)escoedrrx2 << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)escoedrrx2 << 5));
}

__INLINE uint8_t bt_e_scoltcntl_2_e_scoedrtx_2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_e_scoltcntl_2_e_scoedrtx_2_setf(uint8_t escoedrtx2)
{
    ASSERT_ERR((((uint32_t)escoedrtx2 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)escoedrtx2 << 4));
}

__INLINE uint8_t bt_e_scoltcntl_2_syntype2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_e_scoltcntl_2_syntype2_setf(uint8_t syntype2)
{
    ASSERT_ERR((((uint32_t)syntype2 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)syntype2 << 3));
}

__INLINE uint8_t bt_e_scoltcntl_2_synltaddr2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_e_scoltcntl_2_synltaddr2_setf(uint8_t synltaddr2)
{
    ASSERT_ERR((((uint32_t)synltaddr2 << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_E_SCOLTCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOLTCNTL_2_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)synltaddr2 << 0));
}

/**
 * @brief E_SCOTRCNTL_2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31              TXSEQN2   0
 *  29:20               TXLEN2   0x0
 *  19:16              TXTYPE2   0x0
 *  13:04               RXLEN2   0x0
 *  03:00              RXTYPE2   0x0
 * </pre>
 */
#define BT_E_SCOTRCNTL_2_ADDR   0x50090664
#define BT_E_SCOTRCNTL_2_OFFSET 0x00000264
#define BT_E_SCOTRCNTL_2_INDEX  0x00000099
#define BT_E_SCOTRCNTL_2_RESET  0x00000000

__INLINE uint32_t bt_e_scotrcntl_2_get(void)
{
    return REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR);
}

__INLINE void bt_e_scotrcntl_2_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCOTRCNTL_2_ADDR, value);
}

// field definitions
#define BT_TXSEQN2_BIT    ((uint32_t)0x80000000)
#define BT_TXSEQN2_POS    31
#define BT_TXLEN2_MASK    ((uint32_t)0x3FF00000)
#define BT_TXLEN2_LSB     20
#define BT_TXLEN2_WIDTH   ((uint32_t)0x0000000A)
#define BT_TXTYPE2_MASK   ((uint32_t)0x000F0000)
#define BT_TXTYPE2_LSB    16
#define BT_TXTYPE2_WIDTH  ((uint32_t)0x00000004)
#define BT_RXLEN2_MASK    ((uint32_t)0x00003FF0)
#define BT_RXLEN2_LSB     4
#define BT_RXLEN2_WIDTH   ((uint32_t)0x0000000A)
#define BT_RXTYPE2_MASK   ((uint32_t)0x0000000F)
#define BT_RXTYPE2_LSB    0
#define BT_RXTYPE2_WIDTH  ((uint32_t)0x00000004)

#define BT_TXSEQN2_RST    0x0
#define BT_TXLEN2_RST     0x0
#define BT_TXTYPE2_RST    0x0
#define BT_RXLEN2_RST     0x0
#define BT_RXTYPE2_RST    0x0

__INLINE void bt_e_scotrcntl_2_pack(uint8_t txseqn2, uint16_t txlen2, uint8_t txtype2, uint16_t rxlen2, uint8_t rxtype2)
{
    ASSERT_ERR((((uint32_t)txseqn2 << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)txlen2 << 20) & ~((uint32_t)0x3FF00000)) == 0);
    ASSERT_ERR((((uint32_t)txtype2 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxlen2 << 4) & ~((uint32_t)0x00003FF0)) == 0);
    ASSERT_ERR((((uint32_t)rxtype2 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_2_ADDR, ((uint32_t)txseqn2 << 31) | ((uint32_t)txlen2 << 20) | ((uint32_t)txtype2 << 16) | ((uint32_t)rxlen2 << 4) | ((uint32_t)rxtype2 << 0));
}

__INLINE void bt_e_scotrcntl_2_unpack(uint8_t *txseqn2, uint16_t *txlen2, uint8_t *txtype2, uint16_t *rxlen2, uint8_t *rxtype2)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR);

    *txseqn2 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *txlen2 = (localVal & ((uint32_t)0x3FF00000)) >> 20;
    *txtype2 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rxlen2 = (localVal & ((uint32_t)0x00003FF0)) >> 4;
    *rxtype2 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bt_e_scotrcntl_2_txseqn2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_e_scotrcntl_2_txseqn2_setf(uint8_t txseqn2)
{
    ASSERT_ERR((((uint32_t)txseqn2 << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)txseqn2 << 31));
}

__INLINE uint16_t bt_e_scotrcntl_2_txlen2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x3FF00000)) >> 20);
}

__INLINE void bt_e_scotrcntl_2_txlen2_setf(uint16_t txlen2)
{
    ASSERT_ERR((((uint32_t)txlen2 << 20) & ~((uint32_t)0x3FF00000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR) & ~((uint32_t)0x3FF00000)) | ((uint32_t)txlen2 << 20));
}

__INLINE uint8_t bt_e_scotrcntl_2_txtype2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bt_e_scotrcntl_2_txtype2_setf(uint8_t txtype2)
{
    ASSERT_ERR((((uint32_t)txtype2 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)txtype2 << 16));
}

__INLINE uint16_t bt_e_scotrcntl_2_rxlen2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x00003FF0)) >> 4);
}

__INLINE void bt_e_scotrcntl_2_rxlen2_setf(uint16_t rxlen2)
{
    ASSERT_ERR((((uint32_t)rxlen2 << 4) & ~((uint32_t)0x00003FF0)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR) & ~((uint32_t)0x00003FF0)) | ((uint32_t)rxlen2 << 4));
}

__INLINE uint8_t bt_e_scotrcntl_2_rxtype2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bt_e_scotrcntl_2_rxtype2_setf(uint8_t rxtype2)
{
    ASSERT_ERR((((uint32_t)rxtype2 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BT_WR(BT_E_SCOTRCNTL_2_ADDR, (REG_BT_RD(BT_E_SCOTRCNTL_2_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)rxtype2 << 0));
}

/**
 * @brief E_SCODAYCNT_2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  10:00          DAYCOUNTER2   0x0
 * </pre>
 */
#define BT_E_SCODAYCNT_2_ADDR   0x50090668
#define BT_E_SCODAYCNT_2_OFFSET 0x00000268
#define BT_E_SCODAYCNT_2_INDEX  0x0000009A
#define BT_E_SCODAYCNT_2_RESET  0x00000000

__INLINE uint32_t bt_e_scodaycnt_2_get(void)
{
    return REG_BT_RD(BT_E_SCODAYCNT_2_ADDR);
}

__INLINE void bt_e_scodaycnt_2_set(uint32_t value)
{
    REG_BT_WR(BT_E_SCODAYCNT_2_ADDR, value);
}

// field definitions
#define BT_DAYCOUNTER2_MASK   ((uint32_t)0x000007FF)
#define BT_DAYCOUNTER2_LSB    0
#define BT_DAYCOUNTER2_WIDTH  ((uint32_t)0x0000000B)

#define BT_DAYCOUNTER2_RST    0x0

__INLINE uint16_t bt_e_scodaycnt_2_daycounter2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_E_SCODAYCNT_2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000007FF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_e_scodaycnt_2_daycounter2_setf(uint16_t daycounter2)
{
    ASSERT_ERR((((uint32_t)daycounter2 << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_BT_WR(BT_E_SCODAYCNT_2_ADDR, (uint32_t)daycounter2 << 0);
}

/**
 * @brief AUDIOCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:20       LINEAR_FORMAT0   0x3
 *  17:16         SAMPLE_TYPE0   0x1
 *     15             AULAWEN0   0
 *  11:08          AULAW_CODE0   0x0
 *     07              CVSDEN0   0
 *     00       CVSD_BITORDER0   0
 * </pre>
 */
#define BT_AUDIOCNTL0_ADDR   0x50090670
#define BT_AUDIOCNTL0_OFFSET 0x00000270
#define BT_AUDIOCNTL0_INDEX  0x0000009C
#define BT_AUDIOCNTL0_RESET  0x00310000

__INLINE uint32_t bt_audiocntl0_get(void)
{
    return REG_BT_RD(BT_AUDIOCNTL0_ADDR);
}

__INLINE void bt_audiocntl0_set(uint32_t value)
{
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, value);
}

// field definitions
#define BT_LINEAR_FORMAT0_MASK   ((uint32_t)0x00300000)
#define BT_LINEAR_FORMAT0_LSB    20
#define BT_LINEAR_FORMAT0_WIDTH  ((uint32_t)0x00000002)
#define BT_SAMPLE_TYPE0_MASK     ((uint32_t)0x00030000)
#define BT_SAMPLE_TYPE0_LSB      16
#define BT_SAMPLE_TYPE0_WIDTH    ((uint32_t)0x00000002)
#define BT_AULAWEN0_BIT          ((uint32_t)0x00008000)
#define BT_AULAWEN0_POS          15
#define BT_AULAW_CODE0_MASK      ((uint32_t)0x00000F00)
#define BT_AULAW_CODE0_LSB       8
#define BT_AULAW_CODE0_WIDTH     ((uint32_t)0x00000004)
#define BT_CVSDEN0_BIT           ((uint32_t)0x00000080)
#define BT_CVSDEN0_POS           7
#define BT_CVSD_BITORDER0_BIT    ((uint32_t)0x00000001)
#define BT_CVSD_BITORDER0_POS    0

#define BT_LINEAR_FORMAT0_RST    0x3
#define BT_SAMPLE_TYPE0_RST      0x1
#define BT_AULAWEN0_RST          0x0
#define BT_AULAW_CODE0_RST       0x0
#define BT_CVSDEN0_RST           0x0
#define BT_CVSD_BITORDER0_RST    0x0

__INLINE void bt_audiocntl0_pack(uint8_t linearformat0, uint8_t sampletype0, uint8_t aulawen0, uint8_t aulawcode0, uint8_t cvsden0, uint8_t cvsdbitorder0)
{
    ASSERT_ERR((((uint32_t)linearformat0 << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)sampletype0 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)aulawen0 << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)aulawcode0 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)cvsden0 << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)cvsdbitorder0 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, ((uint32_t)linearformat0 << 20) | ((uint32_t)sampletype0 << 16) | ((uint32_t)aulawen0 << 15) | ((uint32_t)aulawcode0 << 8) | ((uint32_t)cvsden0 << 7) | ((uint32_t)cvsdbitorder0 << 0));
}

__INLINE void bt_audiocntl0_unpack(uint8_t *linearformat0, uint8_t *sampletype0, uint8_t *aulawen0, uint8_t *aulawcode0, uint8_t *cvsden0, uint8_t *cvsdbitorder0)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL0_ADDR);

    *linearformat0 = (localVal & ((uint32_t)0x00300000)) >> 20;
    *sampletype0 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *aulawen0 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *aulawcode0 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *cvsden0 = (localVal & ((uint32_t)0x00000080)) >> 7;
    *cvsdbitorder0 = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_audiocntl0_linear_format0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void bt_audiocntl0_linear_format0_setf(uint8_t linearformat0)
{
    ASSERT_ERR((((uint32_t)linearformat0 << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, (REG_BT_RD(BT_AUDIOCNTL0_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)linearformat0 << 20));
}

__INLINE uint8_t bt_audiocntl0_sample_type0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_audiocntl0_sample_type0_setf(uint8_t sampletype0)
{
    ASSERT_ERR((((uint32_t)sampletype0 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, (REG_BT_RD(BT_AUDIOCNTL0_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)sampletype0 << 16));
}

__INLINE uint8_t bt_audiocntl0_aulawen0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_audiocntl0_aulawen0_setf(uint8_t aulawen0)
{
    ASSERT_ERR((((uint32_t)aulawen0 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, (REG_BT_RD(BT_AUDIOCNTL0_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)aulawen0 << 15));
}

__INLINE uint8_t bt_audiocntl0_aulaw_code0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_audiocntl0_aulaw_code0_setf(uint8_t aulawcode0)
{
    ASSERT_ERR((((uint32_t)aulawcode0 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, (REG_BT_RD(BT_AUDIOCNTL0_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)aulawcode0 << 8));
}

__INLINE uint8_t bt_audiocntl0_cvsden0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_audiocntl0_cvsden0_setf(uint8_t cvsden0)
{
    ASSERT_ERR((((uint32_t)cvsden0 << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, (REG_BT_RD(BT_AUDIOCNTL0_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)cvsden0 << 7));
}

__INLINE uint8_t bt_audiocntl0_cvsd_bitorder0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_audiocntl0_cvsd_bitorder0_setf(uint8_t cvsdbitorder0)
{
    ASSERT_ERR((((uint32_t)cvsdbitorder0 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL0_ADDR, (REG_BT_RD(BT_AUDIOCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)cvsdbitorder0 << 0));
}

/**
 * @brief AUDIOCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:20       LINEAR_FORMAT1   0x3
 *  17:16         SAMPLE_TYPE1   0x1
 *     15             AULAWEN1   0
 *  11:08          AULAW_CODE1   0x0
 *     07              CVSDEN1   0
 *     00       CVSD_BITORDER1   0
 * </pre>
 */
#define BT_AUDIOCNTL1_ADDR   0x50090674
#define BT_AUDIOCNTL1_OFFSET 0x00000274
#define BT_AUDIOCNTL1_INDEX  0x0000009D
#define BT_AUDIOCNTL1_RESET  0x00310000

__INLINE uint32_t bt_audiocntl1_get(void)
{
    return REG_BT_RD(BT_AUDIOCNTL1_ADDR);
}

__INLINE void bt_audiocntl1_set(uint32_t value)
{
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, value);
}

// field definitions
#define BT_LINEAR_FORMAT1_MASK   ((uint32_t)0x00300000)
#define BT_LINEAR_FORMAT1_LSB    20
#define BT_LINEAR_FORMAT1_WIDTH  ((uint32_t)0x00000002)
#define BT_SAMPLE_TYPE1_MASK     ((uint32_t)0x00030000)
#define BT_SAMPLE_TYPE1_LSB      16
#define BT_SAMPLE_TYPE1_WIDTH    ((uint32_t)0x00000002)
#define BT_AULAWEN1_BIT          ((uint32_t)0x00008000)
#define BT_AULAWEN1_POS          15
#define BT_AULAW_CODE1_MASK      ((uint32_t)0x00000F00)
#define BT_AULAW_CODE1_LSB       8
#define BT_AULAW_CODE1_WIDTH     ((uint32_t)0x00000004)
#define BT_CVSDEN1_BIT           ((uint32_t)0x00000080)
#define BT_CVSDEN1_POS           7
#define BT_CVSD_BITORDER1_BIT    ((uint32_t)0x00000001)
#define BT_CVSD_BITORDER1_POS    0

#define BT_LINEAR_FORMAT1_RST    0x3
#define BT_SAMPLE_TYPE1_RST      0x1
#define BT_AULAWEN1_RST          0x0
#define BT_AULAW_CODE1_RST       0x0
#define BT_CVSDEN1_RST           0x0
#define BT_CVSD_BITORDER1_RST    0x0

__INLINE void bt_audiocntl1_pack(uint8_t linearformat1, uint8_t sampletype1, uint8_t aulawen1, uint8_t aulawcode1, uint8_t cvsden1, uint8_t cvsdbitorder1)
{
    ASSERT_ERR((((uint32_t)linearformat1 << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)sampletype1 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)aulawen1 << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)aulawcode1 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)cvsden1 << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)cvsdbitorder1 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, ((uint32_t)linearformat1 << 20) | ((uint32_t)sampletype1 << 16) | ((uint32_t)aulawen1 << 15) | ((uint32_t)aulawcode1 << 8) | ((uint32_t)cvsden1 << 7) | ((uint32_t)cvsdbitorder1 << 0));
}

__INLINE void bt_audiocntl1_unpack(uint8_t *linearformat1, uint8_t *sampletype1, uint8_t *aulawen1, uint8_t *aulawcode1, uint8_t *cvsden1, uint8_t *cvsdbitorder1)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL1_ADDR);

    *linearformat1 = (localVal & ((uint32_t)0x00300000)) >> 20;
    *sampletype1 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *aulawen1 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *aulawcode1 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *cvsden1 = (localVal & ((uint32_t)0x00000080)) >> 7;
    *cvsdbitorder1 = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_audiocntl1_linear_format1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void bt_audiocntl1_linear_format1_setf(uint8_t linearformat1)
{
    ASSERT_ERR((((uint32_t)linearformat1 << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, (REG_BT_RD(BT_AUDIOCNTL1_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)linearformat1 << 20));
}

__INLINE uint8_t bt_audiocntl1_sample_type1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_audiocntl1_sample_type1_setf(uint8_t sampletype1)
{
    ASSERT_ERR((((uint32_t)sampletype1 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, (REG_BT_RD(BT_AUDIOCNTL1_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)sampletype1 << 16));
}

__INLINE uint8_t bt_audiocntl1_aulawen1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_audiocntl1_aulawen1_setf(uint8_t aulawen1)
{
    ASSERT_ERR((((uint32_t)aulawen1 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, (REG_BT_RD(BT_AUDIOCNTL1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)aulawen1 << 15));
}

__INLINE uint8_t bt_audiocntl1_aulaw_code1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_audiocntl1_aulaw_code1_setf(uint8_t aulawcode1)
{
    ASSERT_ERR((((uint32_t)aulawcode1 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, (REG_BT_RD(BT_AUDIOCNTL1_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)aulawcode1 << 8));
}

__INLINE uint8_t bt_audiocntl1_cvsden1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_audiocntl1_cvsden1_setf(uint8_t cvsden1)
{
    ASSERT_ERR((((uint32_t)cvsden1 << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, (REG_BT_RD(BT_AUDIOCNTL1_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)cvsden1 << 7));
}

__INLINE uint8_t bt_audiocntl1_cvsd_bitorder1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_audiocntl1_cvsd_bitorder1_setf(uint8_t cvsdbitorder1)
{
    ASSERT_ERR((((uint32_t)cvsdbitorder1 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL1_ADDR, (REG_BT_RD(BT_AUDIOCNTL1_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)cvsdbitorder1 << 0));
}

/**
 * @brief AUDIOCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:20       LINEAR_FORMAT2   0x3
 *  17:16         SAMPLE_TYPE2   0x1
 *     15             AULAWEN2   0
 *  11:08          AULAW_CODE2   0x0
 *     07              CVSDEN2   0
 *     00       CVSD_BITORDER2   0
 * </pre>
 */
#define BT_AUDIOCNTL2_ADDR   0x50090678
#define BT_AUDIOCNTL2_OFFSET 0x00000278
#define BT_AUDIOCNTL2_INDEX  0x0000009E
#define BT_AUDIOCNTL2_RESET  0x00310000

__INLINE uint32_t bt_audiocntl2_get(void)
{
    return REG_BT_RD(BT_AUDIOCNTL2_ADDR);
}

__INLINE void bt_audiocntl2_set(uint32_t value)
{
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, value);
}

// field definitions
#define BT_LINEAR_FORMAT2_MASK   ((uint32_t)0x00300000)
#define BT_LINEAR_FORMAT2_LSB    20
#define BT_LINEAR_FORMAT2_WIDTH  ((uint32_t)0x00000002)
#define BT_SAMPLE_TYPE2_MASK     ((uint32_t)0x00030000)
#define BT_SAMPLE_TYPE2_LSB      16
#define BT_SAMPLE_TYPE2_WIDTH    ((uint32_t)0x00000002)
#define BT_AULAWEN2_BIT          ((uint32_t)0x00008000)
#define BT_AULAWEN2_POS          15
#define BT_AULAW_CODE2_MASK      ((uint32_t)0x00000F00)
#define BT_AULAW_CODE2_LSB       8
#define BT_AULAW_CODE2_WIDTH     ((uint32_t)0x00000004)
#define BT_CVSDEN2_BIT           ((uint32_t)0x00000080)
#define BT_CVSDEN2_POS           7
#define BT_CVSD_BITORDER2_BIT    ((uint32_t)0x00000001)
#define BT_CVSD_BITORDER2_POS    0

#define BT_LINEAR_FORMAT2_RST    0x3
#define BT_SAMPLE_TYPE2_RST      0x1
#define BT_AULAWEN2_RST          0x0
#define BT_AULAW_CODE2_RST       0x0
#define BT_CVSDEN2_RST           0x0
#define BT_CVSD_BITORDER2_RST    0x0

__INLINE void bt_audiocntl2_pack(uint8_t linearformat2, uint8_t sampletype2, uint8_t aulawen2, uint8_t aulawcode2, uint8_t cvsden2, uint8_t cvsdbitorder2)
{
    ASSERT_ERR((((uint32_t)linearformat2 << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)sampletype2 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)aulawen2 << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)aulawcode2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)cvsden2 << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)cvsdbitorder2 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, ((uint32_t)linearformat2 << 20) | ((uint32_t)sampletype2 << 16) | ((uint32_t)aulawen2 << 15) | ((uint32_t)aulawcode2 << 8) | ((uint32_t)cvsden2 << 7) | ((uint32_t)cvsdbitorder2 << 0));
}

__INLINE void bt_audiocntl2_unpack(uint8_t *linearformat2, uint8_t *sampletype2, uint8_t *aulawen2, uint8_t *aulawcode2, uint8_t *cvsden2, uint8_t *cvsdbitorder2)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL2_ADDR);

    *linearformat2 = (localVal & ((uint32_t)0x00300000)) >> 20;
    *sampletype2 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *aulawen2 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *aulawcode2 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *cvsden2 = (localVal & ((uint32_t)0x00000080)) >> 7;
    *cvsdbitorder2 = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_audiocntl2_linear_format2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void bt_audiocntl2_linear_format2_setf(uint8_t linearformat2)
{
    ASSERT_ERR((((uint32_t)linearformat2 << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, (REG_BT_RD(BT_AUDIOCNTL2_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)linearformat2 << 20));
}

__INLINE uint8_t bt_audiocntl2_sample_type2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bt_audiocntl2_sample_type2_setf(uint8_t sampletype2)
{
    ASSERT_ERR((((uint32_t)sampletype2 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, (REG_BT_RD(BT_AUDIOCNTL2_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)sampletype2 << 16));
}

__INLINE uint8_t bt_audiocntl2_aulawen2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bt_audiocntl2_aulawen2_setf(uint8_t aulawen2)
{
    ASSERT_ERR((((uint32_t)aulawen2 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, (REG_BT_RD(BT_AUDIOCNTL2_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)aulawen2 << 15));
}

__INLINE uint8_t bt_audiocntl2_aulaw_code2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bt_audiocntl2_aulaw_code2_setf(uint8_t aulawcode2)
{
    ASSERT_ERR((((uint32_t)aulawcode2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, (REG_BT_RD(BT_AUDIOCNTL2_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)aulawcode2 << 8));
}

__INLINE uint8_t bt_audiocntl2_cvsden2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bt_audiocntl2_cvsden2_setf(uint8_t cvsden2)
{
    ASSERT_ERR((((uint32_t)cvsden2 << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, (REG_BT_RD(BT_AUDIOCNTL2_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)cvsden2 << 7));
}

__INLINE uint8_t bt_audiocntl2_cvsd_bitorder2_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_AUDIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_audiocntl2_cvsd_bitorder2_setf(uint8_t cvsdbitorder2)
{
    ASSERT_ERR((((uint32_t)cvsdbitorder2 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_AUDIOCNTL2_ADDR, (REG_BT_RD(BT_AUDIOCNTL2_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)cvsdbitorder2 << 0));
}

/**
 * @brief PCMGENCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:08              VXCHSEL   0x0
 *     06             LOOPBACK   0
 *     05          MONO_LR_SEL   0
 *     04          MONO_STEREO   0
 *     03               MSTSLV   0
 *     02             BYTESWAP   0
 *     01               LRSWAP   0
 *     00                PCMEN   0
 * </pre>
 */
#define BT_PCMGENCNTL_ADDR   0x50090680
#define BT_PCMGENCNTL_OFFSET 0x00000280
#define BT_PCMGENCNTL_INDEX  0x000000A0
#define BT_PCMGENCNTL_RESET  0x00000000

__INLINE uint32_t bt_pcmgencntl_get(void)
{
    return REG_BT_RD(BT_PCMGENCNTL_ADDR);
}

__INLINE void bt_pcmgencntl_set(uint32_t value)
{
    REG_BT_WR(BT_PCMGENCNTL_ADDR, value);
}

// field definitions
#define BT_VXCHSEL_MASK       ((uint32_t)0x00000300)
#define BT_VXCHSEL_LSB        8
#define BT_VXCHSEL_WIDTH      ((uint32_t)0x00000002)
#define BT_LOOPBACK_BIT       ((uint32_t)0x00000040)
#define BT_LOOPBACK_POS       6
#define BT_MONO_LR_SEL_BIT    ((uint32_t)0x00000020)
#define BT_MONO_LR_SEL_POS    5
#define BT_MONO_STEREO_BIT    ((uint32_t)0x00000010)
#define BT_MONO_STEREO_POS    4
#define BT_MSTSLV_BIT         ((uint32_t)0x00000008)
#define BT_MSTSLV_POS         3
#define BT_BYTESWAP_BIT       ((uint32_t)0x00000004)
#define BT_BYTESWAP_POS       2
#define BT_LRSWAP_BIT         ((uint32_t)0x00000002)
#define BT_LRSWAP_POS         1
#define BT_PCMEN_BIT          ((uint32_t)0x00000001)
#define BT_PCMEN_POS          0

#define BT_VXCHSEL_RST        0x0
#define BT_LOOPBACK_RST       0x0
#define BT_MONO_LR_SEL_RST    0x0
#define BT_MONO_STEREO_RST    0x0
#define BT_MSTSLV_RST         0x0
#define BT_BYTESWAP_RST       0x0
#define BT_LRSWAP_RST         0x0
#define BT_PCMEN_RST          0x0

__INLINE void bt_pcmgencntl_pack(uint8_t vxchsel, uint8_t loopback, uint8_t monolrsel, uint8_t monostereo, uint8_t mstslv, uint8_t byteswap, uint8_t lrswap, uint8_t pcmen)
{
    ASSERT_ERR((((uint32_t)vxchsel << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)loopback << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)monolrsel << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)monostereo << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)mstslv << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)byteswap << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)lrswap << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)pcmen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, ((uint32_t)vxchsel << 8) | ((uint32_t)loopback << 6) | ((uint32_t)monolrsel << 5) | ((uint32_t)monostereo << 4) | ((uint32_t)mstslv << 3) | ((uint32_t)byteswap << 2) | ((uint32_t)lrswap << 1) | ((uint32_t)pcmen << 0));
}

__INLINE void bt_pcmgencntl_unpack(uint8_t *vxchsel, uint8_t *loopback, uint8_t *monolrsel, uint8_t *monostereo, uint8_t *mstslv, uint8_t *byteswap, uint8_t *lrswap, uint8_t *pcmen)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);

    *vxchsel = (localVal & ((uint32_t)0x00000300)) >> 8;
    *loopback = (localVal & ((uint32_t)0x00000040)) >> 6;
    *monolrsel = (localVal & ((uint32_t)0x00000020)) >> 5;
    *monostereo = (localVal & ((uint32_t)0x00000010)) >> 4;
    *mstslv = (localVal & ((uint32_t)0x00000008)) >> 3;
    *byteswap = (localVal & ((uint32_t)0x00000004)) >> 2;
    *lrswap = (localVal & ((uint32_t)0x00000002)) >> 1;
    *pcmen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bt_pcmgencntl_vxchsel_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void bt_pcmgencntl_vxchsel_setf(uint8_t vxchsel)
{
    ASSERT_ERR((((uint32_t)vxchsel << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)vxchsel << 8));
}

__INLINE uint8_t bt_pcmgencntl_loopback_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bt_pcmgencntl_loopback_setf(uint8_t loopback)
{
    ASSERT_ERR((((uint32_t)loopback << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)loopback << 6));
}

__INLINE uint8_t bt_pcmgencntl_mono_lr_sel_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bt_pcmgencntl_mono_lr_sel_setf(uint8_t monolrsel)
{
    ASSERT_ERR((((uint32_t)monolrsel << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)monolrsel << 5));
}

__INLINE uint8_t bt_pcmgencntl_mono_stereo_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bt_pcmgencntl_mono_stereo_setf(uint8_t monostereo)
{
    ASSERT_ERR((((uint32_t)monostereo << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)monostereo << 4));
}

__INLINE uint8_t bt_pcmgencntl_mstslv_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bt_pcmgencntl_mstslv_setf(uint8_t mstslv)
{
    ASSERT_ERR((((uint32_t)mstslv << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)mstslv << 3));
}

__INLINE uint8_t bt_pcmgencntl_byteswap_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bt_pcmgencntl_byteswap_setf(uint8_t byteswap)
{
    ASSERT_ERR((((uint32_t)byteswap << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)byteswap << 2));
}

__INLINE uint8_t bt_pcmgencntl_lrswap_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bt_pcmgencntl_lrswap_setf(uint8_t lrswap)
{
    ASSERT_ERR((((uint32_t)lrswap << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)lrswap << 1));
}

__INLINE uint8_t bt_pcmgencntl_pcmen_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bt_pcmgencntl_pcmen_setf(uint8_t pcmen)
{
    ASSERT_ERR((((uint32_t)pcmen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(BT_PCMGENCNTL_ADDR, (REG_BT_RD(BT_PCMGENCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pcmen << 0));
}

/**
 * @brief PCMPHYSCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:20         FIRSTACTSLOT   0x0
 *  18:16               SLOTNB   0x0
 *     13             SAMPTYPE   0
 *     12               SAMPSZ   0
 *     10               LSB1ST   0
 *     09              PCM_IOM   0
 *     08              LRCHPOL   0
 *  05:04              DOUTCFG   0x0
 *  02:00             FSYNCSHP   0x0
 * </pre>
 */
#define BT_PCMPHYSCNTL0_ADDR   0x50090684
#define BT_PCMPHYSCNTL0_OFFSET 0x00000284
#define BT_PCMPHYSCNTL0_INDEX  0x000000A1
#define BT_PCMPHYSCNTL0_RESET  0x00000000

__INLINE uint32_t bt_pcmphyscntl0_get(void)
{
    return REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
}

__INLINE void bt_pcmphyscntl0_set(uint32_t value)
{
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, value);
}

// field definitions
#define BT_FIRSTACTSLOT_MASK   ((uint32_t)0x00300000)
#define BT_FIRSTACTSLOT_LSB    20
#define BT_FIRSTACTSLOT_WIDTH  ((uint32_t)0x00000002)
#define BT_SLOTNB_MASK         ((uint32_t)0x00070000)
#define BT_SLOTNB_LSB          16
#define BT_SLOTNB_WIDTH        ((uint32_t)0x00000003)
#define BT_SAMPTYPE_BIT        ((uint32_t)0x00002000)
#define BT_SAMPTYPE_POS        13
#define BT_SAMPSZ_BIT          ((uint32_t)0x00001000)
#define BT_SAMPSZ_POS          12
#define BT_LSB1ST_BIT          ((uint32_t)0x00000400)
#define BT_LSB1ST_POS          10
#define BT_PCM_IOM_BIT         ((uint32_t)0x00000200)
#define BT_PCM_IOM_POS         9
#define BT_LRCHPOL_BIT         ((uint32_t)0x00000100)
#define BT_LRCHPOL_POS         8
#define BT_DOUTCFG_MASK        ((uint32_t)0x00000030)
#define BT_DOUTCFG_LSB         4
#define BT_DOUTCFG_WIDTH       ((uint32_t)0x00000002)
#define BT_FSYNCSHP_MASK       ((uint32_t)0x00000007)
#define BT_FSYNCSHP_LSB        0
#define BT_FSYNCSHP_WIDTH      ((uint32_t)0x00000003)

#define BT_FIRSTACTSLOT_RST    0x0
#define BT_SLOTNB_RST          0x0
#define BT_SAMPTYPE_RST        0x0
#define BT_SAMPSZ_RST          0x0
#define BT_LSB1ST_RST          0x0
#define BT_PCM_IOM_RST         0x0
#define BT_LRCHPOL_RST         0x0
#define BT_DOUTCFG_RST         0x0
#define BT_FSYNCSHP_RST        0x0

__INLINE void bt_pcmphyscntl0_pack(uint8_t firstactslot, uint8_t slotnb, uint8_t samptype, uint8_t sampsz, uint8_t lsb1st, uint8_t pcmiom, uint8_t lrchpol, uint8_t doutcfg, uint8_t fsyncshp)
{
    ASSERT_ERR((((uint32_t)firstactslot << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)slotnb << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)samptype << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)sampsz << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)lsb1st << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)pcmiom << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)lrchpol << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)doutcfg << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)fsyncshp << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, ((uint32_t)firstactslot << 20) | ((uint32_t)slotnb << 16) | ((uint32_t)samptype << 13) | ((uint32_t)sampsz << 12) | ((uint32_t)lsb1st << 10) | ((uint32_t)pcmiom << 9) | ((uint32_t)lrchpol << 8) | ((uint32_t)doutcfg << 4) | ((uint32_t)fsyncshp << 0));
}

__INLINE void bt_pcmphyscntl0_unpack(uint8_t *firstactslot, uint8_t *slotnb, uint8_t *samptype, uint8_t *sampsz, uint8_t *lsb1st, uint8_t *pcmiom, uint8_t *lrchpol, uint8_t *doutcfg, uint8_t *fsyncshp)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);

    *firstactslot = (localVal & ((uint32_t)0x00300000)) >> 20;
    *slotnb = (localVal & ((uint32_t)0x00070000)) >> 16;
    *samptype = (localVal & ((uint32_t)0x00002000)) >> 13;
    *sampsz = (localVal & ((uint32_t)0x00001000)) >> 12;
    *lsb1st = (localVal & ((uint32_t)0x00000400)) >> 10;
    *pcmiom = (localVal & ((uint32_t)0x00000200)) >> 9;
    *lrchpol = (localVal & ((uint32_t)0x00000100)) >> 8;
    *doutcfg = (localVal & ((uint32_t)0x00000030)) >> 4;
    *fsyncshp = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bt_pcmphyscntl0_firstactslot_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void bt_pcmphyscntl0_firstactslot_setf(uint8_t firstactslot)
{
    ASSERT_ERR((((uint32_t)firstactslot << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)firstactslot << 20));
}

__INLINE uint8_t bt_pcmphyscntl0_slotnb_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void bt_pcmphyscntl0_slotnb_setf(uint8_t slotnb)
{
    ASSERT_ERR((((uint32_t)slotnb << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)slotnb << 16));
}

__INLINE uint8_t bt_pcmphyscntl0_samptype_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bt_pcmphyscntl0_samptype_setf(uint8_t samptype)
{
    ASSERT_ERR((((uint32_t)samptype << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)samptype << 13));
}

__INLINE uint8_t bt_pcmphyscntl0_sampsz_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bt_pcmphyscntl0_sampsz_setf(uint8_t sampsz)
{
    ASSERT_ERR((((uint32_t)sampsz << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)sampsz << 12));
}

__INLINE uint8_t bt_pcmphyscntl0_lsb1st_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bt_pcmphyscntl0_lsb1st_setf(uint8_t lsb1st)
{
    ASSERT_ERR((((uint32_t)lsb1st << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)lsb1st << 10));
}

__INLINE uint8_t bt_pcmphyscntl0_pcm_iom_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void bt_pcmphyscntl0_pcm_iom_setf(uint8_t pcmiom)
{
    ASSERT_ERR((((uint32_t)pcmiom << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)pcmiom << 9));
}

__INLINE uint8_t bt_pcmphyscntl0_lrchpol_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bt_pcmphyscntl0_lrchpol_setf(uint8_t lrchpol)
{
    ASSERT_ERR((((uint32_t)lrchpol << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)lrchpol << 8));
}

__INLINE uint8_t bt_pcmphyscntl0_doutcfg_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void bt_pcmphyscntl0_doutcfg_setf(uint8_t doutcfg)
{
    ASSERT_ERR((((uint32_t)doutcfg << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)doutcfg << 4));
}

__INLINE uint8_t bt_pcmphyscntl0_fsyncshp_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bt_pcmphyscntl0_fsyncshp_setf(uint8_t fsyncshp)
{
    ASSERT_ERR((((uint32_t)fsyncshp << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL0_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)fsyncshp << 0));
}

/**
 * @brief PCMPHYSCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31               CLKINV   0
 *  23:16          PCMCLKLIMIT   0x0
 *  08:00            PCMCLKVAL   0x0
 * </pre>
 */
#define BT_PCMPHYSCNTL1_ADDR   0x50090688
#define BT_PCMPHYSCNTL1_OFFSET 0x00000288
#define BT_PCMPHYSCNTL1_INDEX  0x000000A2
#define BT_PCMPHYSCNTL1_RESET  0x00000000

__INLINE uint32_t bt_pcmphyscntl1_get(void)
{
    return REG_BT_RD(BT_PCMPHYSCNTL1_ADDR);
}

__INLINE void bt_pcmphyscntl1_set(uint32_t value)
{
    REG_BT_WR(BT_PCMPHYSCNTL1_ADDR, value);
}

// field definitions
#define BT_CLKINV_BIT         ((uint32_t)0x80000000)
#define BT_CLKINV_POS         31
#define BT_PCMCLKLIMIT_MASK   ((uint32_t)0x00FF0000)
#define BT_PCMCLKLIMIT_LSB    16
#define BT_PCMCLKLIMIT_WIDTH  ((uint32_t)0x00000008)
#define BT_PCMCLKVAL_MASK     ((uint32_t)0x000001FF)
#define BT_PCMCLKVAL_LSB      0
#define BT_PCMCLKVAL_WIDTH    ((uint32_t)0x00000009)

#define BT_CLKINV_RST         0x0
#define BT_PCMCLKLIMIT_RST    0x0
#define BT_PCMCLKVAL_RST      0x0

__INLINE void bt_pcmphyscntl1_pack(uint8_t clkinv, uint8_t pcmclklimit, uint16_t pcmclkval)
{
    ASSERT_ERR((((uint32_t)clkinv << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)pcmclklimit << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)pcmclkval << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL1_ADDR, ((uint32_t)clkinv << 31) | ((uint32_t)pcmclklimit << 16) | ((uint32_t)pcmclkval << 0));
}

__INLINE void bt_pcmphyscntl1_unpack(uint8_t *clkinv, uint8_t *pcmclklimit, uint16_t *pcmclkval)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL1_ADDR);

    *clkinv = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pcmclklimit = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *pcmclkval = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint8_t bt_pcmphyscntl1_clkinv_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bt_pcmphyscntl1_clkinv_setf(uint8_t clkinv)
{
    ASSERT_ERR((((uint32_t)clkinv << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL1_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)clkinv << 31));
}

__INLINE uint8_t bt_pcmphyscntl1_pcmclklimit_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bt_pcmphyscntl1_pcmclklimit_setf(uint8_t pcmclklimit)
{
    ASSERT_ERR((((uint32_t)pcmclklimit << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL1_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)pcmclklimit << 16));
}

__INLINE uint16_t bt_pcmphyscntl1_pcmclkval_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPHYSCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void bt_pcmphyscntl1_pcmclkval_setf(uint16_t pcmclkval)
{
    ASSERT_ERR((((uint32_t)pcmclkval << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BT_WR(BT_PCMPHYSCNTL1_ADDR, (REG_BT_RD(BT_PCMPHYSCNTL1_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)pcmclkval << 0));
}

/**
 * @brief PCMPADDING register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16             RSAMPPAD   0x0
 *  15:00             LSAMPPAD   0x0
 * </pre>
 */
#define BT_PCMPADDING_ADDR   0x5009068C
#define BT_PCMPADDING_OFFSET 0x0000028C
#define BT_PCMPADDING_INDEX  0x000000A3
#define BT_PCMPADDING_RESET  0x00000000

__INLINE uint32_t bt_pcmpadding_get(void)
{
    return REG_BT_RD(BT_PCMPADDING_ADDR);
}

__INLINE void bt_pcmpadding_set(uint32_t value)
{
    REG_BT_WR(BT_PCMPADDING_ADDR, value);
}

// field definitions
#define BT_RSAMPPAD_MASK   ((uint32_t)0xFFFF0000)
#define BT_RSAMPPAD_LSB    16
#define BT_RSAMPPAD_WIDTH  ((uint32_t)0x00000010)
#define BT_LSAMPPAD_MASK   ((uint32_t)0x0000FFFF)
#define BT_LSAMPPAD_LSB    0
#define BT_LSAMPPAD_WIDTH  ((uint32_t)0x00000010)

#define BT_RSAMPPAD_RST    0x0
#define BT_LSAMPPAD_RST    0x0

__INLINE void bt_pcmpadding_pack(uint16_t rsamppad, uint16_t lsamppad)
{
    ASSERT_ERR((((uint32_t)rsamppad << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)lsamppad << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_PCMPADDING_ADDR, ((uint32_t)rsamppad << 16) | ((uint32_t)lsamppad << 0));
}

__INLINE void bt_pcmpadding_unpack(uint16_t *rsamppad, uint16_t *lsamppad)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPADDING_ADDR);

    *rsamppad = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *lsamppad = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t bt_pcmpadding_rsamppad_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPADDING_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bt_pcmpadding_rsamppad_setf(uint16_t rsamppad)
{
    ASSERT_ERR((((uint32_t)rsamppad << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BT_WR(BT_PCMPADDING_ADDR, (REG_BT_RD(BT_PCMPADDING_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)rsamppad << 16));
}

__INLINE uint16_t bt_pcmpadding_lsamppad_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPADDING_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void bt_pcmpadding_lsamppad_setf(uint16_t lsamppad)
{
    ASSERT_ERR((((uint32_t)lsamppad << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(BT_PCMPADDING_ADDR, (REG_BT_RD(BT_PCMPADDING_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)lsamppad << 0));
}

/**
 * @brief PCMPLLCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00                   RL   0x0
 * </pre>
 */
#define BT_PCMPLLCNTL0_ADDR   0x50090690
#define BT_PCMPLLCNTL0_OFFSET 0x00000290
#define BT_PCMPLLCNTL0_INDEX  0x000000A4
#define BT_PCMPLLCNTL0_RESET  0x00000000

__INLINE uint32_t bt_pcmpllcntl0_get(void)
{
    return REG_BT_RD(BT_PCMPLLCNTL0_ADDR);
}

__INLINE void bt_pcmpllcntl0_set(uint32_t value)
{
    REG_BT_WR(BT_PCMPLLCNTL0_ADDR, value);
}

// field definitions
#define BT_RL_MASK   ((uint32_t)0x000FFFFF)
#define BT_RL_LSB    0
#define BT_RL_WIDTH  ((uint32_t)0x00000014)

#define BT_RL_RST    0x0

__INLINE uint32_t bt_pcmpllcntl0_rl_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPLLCNTL0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000FFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_pcmpllcntl0_rl_setf(uint32_t rl)
{
    ASSERT_ERR((((uint32_t)rl << 0) & ~((uint32_t)0x000FFFFF)) == 0);
    REG_BT_WR(BT_PCMPLLCNTL0_ADDR, (uint32_t)rl << 0);
}

/**
 * @brief PCMPLLCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:20                  OLC   0x0
 *  18:00                    A   0x0
 * </pre>
 */
#define BT_PCMPLLCNTL1_ADDR   0x50090694
#define BT_PCMPLLCNTL1_OFFSET 0x00000294
#define BT_PCMPLLCNTL1_INDEX  0x000000A5
#define BT_PCMPLLCNTL1_RESET  0x00000000

__INLINE uint32_t bt_pcmpllcntl1_get(void)
{
    return REG_BT_RD(BT_PCMPLLCNTL1_ADDR);
}

__INLINE void bt_pcmpllcntl1_set(uint32_t value)
{
    REG_BT_WR(BT_PCMPLLCNTL1_ADDR, value);
}

// field definitions
#define BT_OLC_MASK   ((uint32_t)0x7FF00000)
#define BT_OLC_LSB    20
#define BT_OLC_WIDTH  ((uint32_t)0x0000000B)
#define BT_A_MASK     ((uint32_t)0x0007FFFF)
#define BT_A_LSB      0
#define BT_A_WIDTH    ((uint32_t)0x00000013)

#define BT_OLC_RST    0x0
#define BT_A_RST      0x0

__INLINE void bt_pcmpllcntl1_pack(uint16_t olc, uint32_t a)
{
    ASSERT_ERR((((uint32_t)olc << 20) & ~((uint32_t)0x7FF00000)) == 0);
    ASSERT_ERR((((uint32_t)a << 0) & ~((uint32_t)0x0007FFFF)) == 0);
    REG_BT_WR(BT_PCMPLLCNTL1_ADDR, ((uint32_t)olc << 20) | ((uint32_t)a << 0));
}

__INLINE void bt_pcmpllcntl1_unpack(uint16_t *olc, uint32_t *a)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPLLCNTL1_ADDR);

    *olc = (localVal & ((uint32_t)0x7FF00000)) >> 20;
    *a = (localVal & ((uint32_t)0x0007FFFF)) >> 0;
}

__INLINE uint16_t bt_pcmpllcntl1_olc_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x7FF00000)) >> 20);
}

__INLINE void bt_pcmpllcntl1_olc_setf(uint16_t olc)
{
    ASSERT_ERR((((uint32_t)olc << 20) & ~((uint32_t)0x7FF00000)) == 0);
    REG_BT_WR(BT_PCMPLLCNTL1_ADDR, (REG_BT_RD(BT_PCMPLLCNTL1_ADDR) & ~((uint32_t)0x7FF00000)) | ((uint32_t)olc << 20));
}

__INLINE uint32_t bt_pcmpllcntl1_a_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0007FFFF)) >> 0);
}

__INLINE void bt_pcmpllcntl1_a_setf(uint32_t a)
{
    ASSERT_ERR((((uint32_t)a << 0) & ~((uint32_t)0x0007FFFF)) == 0);
    REG_BT_WR(BT_PCMPLLCNTL1_ADDR, (REG_BT_RD(BT_PCMPLLCNTL1_ADDR) & ~((uint32_t)0x0007FFFF)) | ((uint32_t)a << 0));
}

/**
 * @brief PCMPLLCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  18:00                    W   0x0
 * </pre>
 */
#define BT_PCMPLLCNTL2_ADDR   0x50090698
#define BT_PCMPLLCNTL2_OFFSET 0x00000298
#define BT_PCMPLLCNTL2_INDEX  0x000000A6
#define BT_PCMPLLCNTL2_RESET  0x00000000

__INLINE uint32_t bt_pcmpllcntl2_get(void)
{
    return REG_BT_RD(BT_PCMPLLCNTL2_ADDR);
}

__INLINE void bt_pcmpllcntl2_set(uint32_t value)
{
    REG_BT_WR(BT_PCMPLLCNTL2_ADDR, value);
}

// field definitions
#define BT_W_MASK   ((uint32_t)0x0007FFFF)
#define BT_W_LSB    0
#define BT_W_WIDTH  ((uint32_t)0x00000013)

#define BT_W_RST    0x0

__INLINE uint32_t bt_pcmpllcntl2_w_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMPLLCNTL2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0007FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void bt_pcmpllcntl2_w_setf(uint32_t w)
{
    ASSERT_ERR((((uint32_t)w << 0) & ~((uint32_t)0x0007FFFF)) == 0);
    REG_BT_WR(BT_PCMPLLCNTL2_ADDR, (uint32_t)w << 0);
}

/**
 * @brief PCMSOURCEPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16        PCMSOURCEPTR1   0x0
 *  13:00        PCMSOURCEPTR0   0x0
 * </pre>
 */
#define BT_PCMSOURCEPTR_ADDR   0x5009069C
#define BT_PCMSOURCEPTR_OFFSET 0x0000029C
#define BT_PCMSOURCEPTR_INDEX  0x000000A7
#define BT_PCMSOURCEPTR_RESET  0x00000000

__INLINE uint32_t bt_pcmsourceptr_get(void)
{
    return REG_BT_RD(BT_PCMSOURCEPTR_ADDR);
}

__INLINE void bt_pcmsourceptr_set(uint32_t value)
{
    REG_BT_WR(BT_PCMSOURCEPTR_ADDR, value);
}

// field definitions
#define BT_PCMSOURCEPTR1_MASK   ((uint32_t)0x3FFF0000)
#define BT_PCMSOURCEPTR1_LSB    16
#define BT_PCMSOURCEPTR1_WIDTH  ((uint32_t)0x0000000E)
#define BT_PCMSOURCEPTR0_MASK   ((uint32_t)0x00003FFF)
#define BT_PCMSOURCEPTR0_LSB    0
#define BT_PCMSOURCEPTR0_WIDTH  ((uint32_t)0x0000000E)

#define BT_PCMSOURCEPTR1_RST    0x0
#define BT_PCMSOURCEPTR0_RST    0x0

__INLINE void bt_pcmsourceptr_pack(uint16_t pcmsourceptr1, uint16_t pcmsourceptr0)
{
    ASSERT_ERR((((uint32_t)pcmsourceptr1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)pcmsourceptr0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_PCMSOURCEPTR_ADDR, ((uint32_t)pcmsourceptr1 << 16) | ((uint32_t)pcmsourceptr0 << 0));
}

__INLINE void bt_pcmsourceptr_unpack(uint16_t *pcmsourceptr1, uint16_t *pcmsourceptr0)
{
    uint32_t localVal = REG_BT_RD(BT_PCMSOURCEPTR_ADDR);

    *pcmsourceptr1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *pcmsourceptr0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_pcmsourceptr_pcmsourceptr1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMSOURCEPTR_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_pcmsourceptr_pcmsourceptr1_setf(uint16_t pcmsourceptr1)
{
    ASSERT_ERR((((uint32_t)pcmsourceptr1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_PCMSOURCEPTR_ADDR, (REG_BT_RD(BT_PCMSOURCEPTR_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)pcmsourceptr1 << 16));
}

__INLINE uint16_t bt_pcmsourceptr_pcmsourceptr0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMSOURCEPTR_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_pcmsourceptr_pcmsourceptr0_setf(uint16_t pcmsourceptr0)
{
    ASSERT_ERR((((uint32_t)pcmsourceptr0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_PCMSOURCEPTR_ADDR, (REG_BT_RD(BT_PCMSOURCEPTR_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)pcmsourceptr0 << 0));
}

/**
 * @brief PCMSINKPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16          PCMSINKPTR1   0x0
 *  13:00          PCMSINKPTR0   0x0
 * </pre>
 */
#define BT_PCMSINKPTR_ADDR   0x500906A0
#define BT_PCMSINKPTR_OFFSET 0x000002A0
#define BT_PCMSINKPTR_INDEX  0x000000A8
#define BT_PCMSINKPTR_RESET  0x00000000

__INLINE uint32_t bt_pcmsinkptr_get(void)
{
    return REG_BT_RD(BT_PCMSINKPTR_ADDR);
}

__INLINE void bt_pcmsinkptr_set(uint32_t value)
{
    REG_BT_WR(BT_PCMSINKPTR_ADDR, value);
}

// field definitions
#define BT_PCMSINKPTR1_MASK   ((uint32_t)0x3FFF0000)
#define BT_PCMSINKPTR1_LSB    16
#define BT_PCMSINKPTR1_WIDTH  ((uint32_t)0x0000000E)
#define BT_PCMSINKPTR0_MASK   ((uint32_t)0x00003FFF)
#define BT_PCMSINKPTR0_LSB    0
#define BT_PCMSINKPTR0_WIDTH  ((uint32_t)0x0000000E)

#define BT_PCMSINKPTR1_RST    0x0
#define BT_PCMSINKPTR0_RST    0x0

__INLINE void bt_pcmsinkptr_pack(uint16_t pcmsinkptr1, uint16_t pcmsinkptr0)
{
    ASSERT_ERR((((uint32_t)pcmsinkptr1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)pcmsinkptr0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_PCMSINKPTR_ADDR, ((uint32_t)pcmsinkptr1 << 16) | ((uint32_t)pcmsinkptr0 << 0));
}

__INLINE void bt_pcmsinkptr_unpack(uint16_t *pcmsinkptr1, uint16_t *pcmsinkptr0)
{
    uint32_t localVal = REG_BT_RD(BT_PCMSINKPTR_ADDR);

    *pcmsinkptr1 = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *pcmsinkptr0 = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t bt_pcmsinkptr_pcmsinkptr1_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMSINKPTR_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE void bt_pcmsinkptr_pcmsinkptr1_setf(uint16_t pcmsinkptr1)
{
    ASSERT_ERR((((uint32_t)pcmsinkptr1 << 16) & ~((uint32_t)0x3FFF0000)) == 0);
    REG_BT_WR(BT_PCMSINKPTR_ADDR, (REG_BT_RD(BT_PCMSINKPTR_ADDR) & ~((uint32_t)0x3FFF0000)) | ((uint32_t)pcmsinkptr1 << 16));
}

__INLINE uint16_t bt_pcmsinkptr_pcmsinkptr0_getf(void)
{
    uint32_t localVal = REG_BT_RD(BT_PCMSINKPTR_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void bt_pcmsinkptr_pcmsinkptr0_setf(uint16_t pcmsinkptr0)
{
    ASSERT_ERR((((uint32_t)pcmsinkptr0 << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BT_WR(BT_PCMSINKPTR_ADDR, (REG_BT_RD(BT_PCMSINKPTR_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)pcmsinkptr0 << 0));
}


#endif // _REG_BTCORE_H_

