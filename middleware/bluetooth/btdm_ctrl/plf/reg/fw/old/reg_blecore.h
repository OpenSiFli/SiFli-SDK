#ifndef _REG_BLECORE_H_
#define _REG_BLECORE_H_

#include <stdint.h>
#include "_reg_blecore.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_BLECORE_COUNT 119

#define REG_BLECORE_DECODING_MASK 0x000001FF

/**
 * @brief RWBLECNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31      MASTER_SOFT_RST   0
 *     30    MASTER_TGSOFT_RST   0
 *     29         REG_SOFT_RST   0
 *     28   RADIOCNTL_SOFT_RST   0
 *     27            SWINT_REQ   0
 *     26         RFTEST_ABORT   0
 *     25         ADVERT_ABORT   0
 *     24           SCAN_ABORT   0
 *     20               MD_DSB   0
 *     19               SN_DSB   0
 *     18             NESN_DSB   0
 *     17            CRYPT_DSB   0
 *     16           LRPMAP_DSB   0
 *     15            LRFEC_DSB   0
 *     14             WHIT_DSB   0
 *     13              CRC_DSB   0
 *     12        HOP_REMAP_DSB   0
 *     10   ANONYMOUS_ADV_FILT_EN   0
 *     09        ADVERTFILT_EN   0
 *     08             RWBLE_EN   0
 *  03:00           RXWINSZDEF   0x0
 * </pre>
 */
#define BLE_RWBLECNTL_ADDR   0x40100000
#define BLE_RWBLECNTL_OFFSET 0x00000000
#define BLE_RWBLECNTL_INDEX  0x00000000
#define BLE_RWBLECNTL_RESET  0x00000000

__INLINE uint32_t ble_rwblecntl_get(void)
{
    return REG_BLE_RD(BLE_RWBLECNTL_ADDR);
}

__INLINE void ble_rwblecntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, value);
}

// field definitions
#define BLE_MASTER_SOFT_RST_BIT          ((uint32_t)0x80000000)
#define BLE_MASTER_SOFT_RST_POS          31
#define BLE_MASTER_TGSOFT_RST_BIT        ((uint32_t)0x40000000)
#define BLE_MASTER_TGSOFT_RST_POS        30
#define BLE_REG_SOFT_RST_BIT             ((uint32_t)0x20000000)
#define BLE_REG_SOFT_RST_POS             29
#define BLE_RADIOCNTL_SOFT_RST_BIT       ((uint32_t)0x10000000)
#define BLE_RADIOCNTL_SOFT_RST_POS       28
#define BLE_SWINT_REQ_BIT                ((uint32_t)0x08000000)
#define BLE_SWINT_REQ_POS                27
#define BLE_RFTEST_ABORT_BIT             ((uint32_t)0x04000000)
#define BLE_RFTEST_ABORT_POS             26
#define BLE_ADVERT_ABORT_BIT             ((uint32_t)0x02000000)
#define BLE_ADVERT_ABORT_POS             25
#define BLE_SCAN_ABORT_BIT               ((uint32_t)0x01000000)
#define BLE_SCAN_ABORT_POS               24
#define BLE_MD_DSB_BIT                   ((uint32_t)0x00100000)
#define BLE_MD_DSB_POS                   20
#define BLE_SN_DSB_BIT                   ((uint32_t)0x00080000)
#define BLE_SN_DSB_POS                   19
#define BLE_NESN_DSB_BIT                 ((uint32_t)0x00040000)
#define BLE_NESN_DSB_POS                 18
#define BLE_CRYPT_DSB_BIT                ((uint32_t)0x00020000)
#define BLE_CRYPT_DSB_POS                17
#define BLE_LRPMAP_DSB_BIT               ((uint32_t)0x00010000)
#define BLE_LRPMAP_DSB_POS               16
#define BLE_LRFEC_DSB_BIT                ((uint32_t)0x00008000)
#define BLE_LRFEC_DSB_POS                15
#define BLE_WHIT_DSB_BIT                 ((uint32_t)0x00004000)
#define BLE_WHIT_DSB_POS                 14
#define BLE_CRC_DSB_BIT                  ((uint32_t)0x00002000)
#define BLE_CRC_DSB_POS                  13
#define BLE_HOP_REMAP_DSB_BIT            ((uint32_t)0x00001000)
#define BLE_HOP_REMAP_DSB_POS            12
#define BLE_ANONYMOUS_ADV_FILT_EN_BIT    ((uint32_t)0x00000400)
#define BLE_ANONYMOUS_ADV_FILT_EN_POS    10
#define BLE_ADVERTFILT_EN_BIT            ((uint32_t)0x00000200)
#define BLE_ADVERTFILT_EN_POS            9
#define BLE_RWBLE_EN_BIT                 ((uint32_t)0x00000100)
#define BLE_RWBLE_EN_POS                 8
#define BLE_RXWINSZDEF_MASK              ((uint32_t)0x0000000F)
#define BLE_RXWINSZDEF_LSB               0
#define BLE_RXWINSZDEF_WIDTH             ((uint32_t)0x00000004)

#define BLE_MASTER_SOFT_RST_RST          0x0
#define BLE_MASTER_TGSOFT_RST_RST        0x0
#define BLE_REG_SOFT_RST_RST             0x0
#define BLE_RADIOCNTL_SOFT_RST_RST       0x0
#define BLE_SWINT_REQ_RST                0x0
#define BLE_RFTEST_ABORT_RST             0x0
#define BLE_ADVERT_ABORT_RST             0x0
#define BLE_SCAN_ABORT_RST               0x0
#define BLE_MD_DSB_RST                   0x0
#define BLE_SN_DSB_RST                   0x0
#define BLE_NESN_DSB_RST                 0x0
#define BLE_CRYPT_DSB_RST                0x0
#define BLE_LRPMAP_DSB_RST               0x0
#define BLE_LRFEC_DSB_RST                0x0
#define BLE_WHIT_DSB_RST                 0x0
#define BLE_CRC_DSB_RST                  0x0
#define BLE_HOP_REMAP_DSB_RST            0x0
#define BLE_ANONYMOUS_ADV_FILT_EN_RST    0x0
#define BLE_ADVERTFILT_EN_RST            0x0
#define BLE_RWBLE_EN_RST                 0x0
#define BLE_RXWINSZDEF_RST               0x0

__INLINE void ble_rwblecntl_pack(uint8_t mastersoftrst, uint8_t mastertgsoftrst, uint8_t regsoftrst, uint8_t radiocntlsoftrst, uint8_t swintreq, uint8_t rftestabort, uint8_t advertabort, uint8_t scanabort, uint8_t mddsb, uint8_t sndsb, uint8_t nesndsb, uint8_t cryptdsb, uint8_t lrpmapdsb, uint8_t lrfecdsb, uint8_t whitdsb, uint8_t crcdsb, uint8_t hopremapdsb, uint8_t anonymousadvfilten, uint8_t advertfilten, uint8_t rwbleen, uint8_t rxwinszdef)
{
    ASSERT_ERR((((uint32_t)mastersoftrst << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)mastertgsoftrst << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)regsoftrst << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)radiocntlsoftrst << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)swintreq << 27) & ~((uint32_t)0x08000000)) == 0);
    ASSERT_ERR((((uint32_t)rftestabort << 26) & ~((uint32_t)0x04000000)) == 0);
    ASSERT_ERR((((uint32_t)advertabort << 25) & ~((uint32_t)0x02000000)) == 0);
    ASSERT_ERR((((uint32_t)scanabort << 24) & ~((uint32_t)0x01000000)) == 0);
    ASSERT_ERR((((uint32_t)mddsb << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)sndsb << 19) & ~((uint32_t)0x00080000)) == 0);
    ASSERT_ERR((((uint32_t)nesndsb << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)cryptdsb << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)lrpmapdsb << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)lrfecdsb << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)whitdsb << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)crcdsb << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)hopremapdsb << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)anonymousadvfilten << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)advertfilten << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)rwbleen << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rxwinszdef << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, ((uint32_t)mastersoftrst << 31) | ((uint32_t)mastertgsoftrst << 30) | ((uint32_t)regsoftrst << 29) | ((uint32_t)radiocntlsoftrst << 28) | ((uint32_t)swintreq << 27) | ((uint32_t)rftestabort << 26) | ((uint32_t)advertabort << 25) | ((uint32_t)scanabort << 24) | ((uint32_t)mddsb << 20) | ((uint32_t)sndsb << 19) | ((uint32_t)nesndsb << 18) | ((uint32_t)cryptdsb << 17) | ((uint32_t)lrpmapdsb << 16) | ((uint32_t)lrfecdsb << 15) | ((uint32_t)whitdsb << 14) | ((uint32_t)crcdsb << 13) | ((uint32_t)hopremapdsb << 12) | ((uint32_t)anonymousadvfilten << 10) | ((uint32_t)advertfilten << 9) | ((uint32_t)rwbleen << 8) | ((uint32_t)rxwinszdef << 0));
}

__INLINE void ble_rwblecntl_unpack(uint8_t *mastersoftrst, uint8_t *mastertgsoftrst, uint8_t *regsoftrst, uint8_t *radiocntlsoftrst, uint8_t *swintreq, uint8_t *rftestabort, uint8_t *advertabort, uint8_t *scanabort, uint8_t *mddsb, uint8_t *sndsb, uint8_t *nesndsb, uint8_t *cryptdsb, uint8_t *lrpmapdsb, uint8_t *lrfecdsb, uint8_t *whitdsb, uint8_t *crcdsb, uint8_t *hopremapdsb, uint8_t *anonymousadvfilten, uint8_t *advertfilten, uint8_t *rwbleen, uint8_t *rxwinszdef)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);

    *mastersoftrst = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mastertgsoftrst = (localVal & ((uint32_t)0x40000000)) >> 30;
    *regsoftrst = (localVal & ((uint32_t)0x20000000)) >> 29;
    *radiocntlsoftrst = (localVal & ((uint32_t)0x10000000)) >> 28;
    *swintreq = (localVal & ((uint32_t)0x08000000)) >> 27;
    *rftestabort = (localVal & ((uint32_t)0x04000000)) >> 26;
    *advertabort = (localVal & ((uint32_t)0x02000000)) >> 25;
    *scanabort = (localVal & ((uint32_t)0x01000000)) >> 24;
    *mddsb = (localVal & ((uint32_t)0x00100000)) >> 20;
    *sndsb = (localVal & ((uint32_t)0x00080000)) >> 19;
    *nesndsb = (localVal & ((uint32_t)0x00040000)) >> 18;
    *cryptdsb = (localVal & ((uint32_t)0x00020000)) >> 17;
    *lrpmapdsb = (localVal & ((uint32_t)0x00010000)) >> 16;
    *lrfecdsb = (localVal & ((uint32_t)0x00008000)) >> 15;
    *whitdsb = (localVal & ((uint32_t)0x00004000)) >> 14;
    *crcdsb = (localVal & ((uint32_t)0x00002000)) >> 13;
    *hopremapdsb = (localVal & ((uint32_t)0x00001000)) >> 12;
    *anonymousadvfilten = (localVal & ((uint32_t)0x00000400)) >> 10;
    *advertfilten = (localVal & ((uint32_t)0x00000200)) >> 9;
    *rwbleen = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxwinszdef = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ble_rwblecntl_master_soft_rst_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_rwblecntl_master_soft_rst_setf(uint8_t mastersoftrst)
{
    ASSERT_ERR((((uint32_t)mastersoftrst << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)mastersoftrst << 31));
}

__INLINE uint8_t ble_rwblecntl_master_tgsoft_rst_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ble_rwblecntl_master_tgsoft_rst_setf(uint8_t mastertgsoftrst)
{
    ASSERT_ERR((((uint32_t)mastertgsoftrst << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)mastertgsoftrst << 30));
}

__INLINE uint8_t ble_rwblecntl_reg_soft_rst_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void ble_rwblecntl_reg_soft_rst_setf(uint8_t regsoftrst)
{
    ASSERT_ERR((((uint32_t)regsoftrst << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)regsoftrst << 29));
}

__INLINE uint8_t ble_rwblecntl_radiocntl_soft_rst_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void ble_rwblecntl_radiocntl_soft_rst_setf(uint8_t radiocntlsoftrst)
{
    ASSERT_ERR((((uint32_t)radiocntlsoftrst << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)radiocntlsoftrst << 28));
}

__INLINE uint8_t ble_rwblecntl_swint_req_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void ble_rwblecntl_swint_req_setf(uint8_t swintreq)
{
    ASSERT_ERR((((uint32_t)swintreq << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)swintreq << 27));
}

__INLINE uint8_t ble_rwblecntl_rftest_abort_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void ble_rwblecntl_rftest_abort_setf(uint8_t rftestabort)
{
    ASSERT_ERR((((uint32_t)rftestabort << 26) & ~((uint32_t)0x04000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)rftestabort << 26));
}

__INLINE uint8_t ble_rwblecntl_advert_abort_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void ble_rwblecntl_advert_abort_setf(uint8_t advertabort)
{
    ASSERT_ERR((((uint32_t)advertabort << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)advertabort << 25));
}

__INLINE uint8_t ble_rwblecntl_scan_abort_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void ble_rwblecntl_scan_abort_setf(uint8_t scanabort)
{
    ASSERT_ERR((((uint32_t)scanabort << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)scanabort << 24));
}

__INLINE uint8_t ble_rwblecntl_md_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void ble_rwblecntl_md_dsb_setf(uint8_t mddsb)
{
    ASSERT_ERR((((uint32_t)mddsb << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)mddsb << 20));
}

__INLINE uint8_t ble_rwblecntl_sn_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void ble_rwblecntl_sn_dsb_setf(uint8_t sndsb)
{
    ASSERT_ERR((((uint32_t)sndsb << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)sndsb << 19));
}

__INLINE uint8_t ble_rwblecntl_nesn_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void ble_rwblecntl_nesn_dsb_setf(uint8_t nesndsb)
{
    ASSERT_ERR((((uint32_t)nesndsb << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)nesndsb << 18));
}

__INLINE uint8_t ble_rwblecntl_crypt_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void ble_rwblecntl_crypt_dsb_setf(uint8_t cryptdsb)
{
    ASSERT_ERR((((uint32_t)cryptdsb << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)cryptdsb << 17));
}

__INLINE uint8_t ble_rwblecntl_lrpmap_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void ble_rwblecntl_lrpmap_dsb_setf(uint8_t lrpmapdsb)
{
    ASSERT_ERR((((uint32_t)lrpmapdsb << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)lrpmapdsb << 16));
}

__INLINE uint8_t ble_rwblecntl_lrfec_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ble_rwblecntl_lrfec_dsb_setf(uint8_t lrfecdsb)
{
    ASSERT_ERR((((uint32_t)lrfecdsb << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)lrfecdsb << 15));
}

__INLINE uint8_t ble_rwblecntl_whit_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ble_rwblecntl_whit_dsb_setf(uint8_t whitdsb)
{
    ASSERT_ERR((((uint32_t)whitdsb << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)whitdsb << 14));
}

__INLINE uint8_t ble_rwblecntl_crc_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ble_rwblecntl_crc_dsb_setf(uint8_t crcdsb)
{
    ASSERT_ERR((((uint32_t)crcdsb << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)crcdsb << 13));
}

__INLINE uint8_t ble_rwblecntl_hop_remap_dsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ble_rwblecntl_hop_remap_dsb_setf(uint8_t hopremapdsb)
{
    ASSERT_ERR((((uint32_t)hopremapdsb << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)hopremapdsb << 12));
}

__INLINE uint8_t ble_rwblecntl_anonymous_adv_filt_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ble_rwblecntl_anonymous_adv_filt_en_setf(uint8_t anonymousadvfilten)
{
    ASSERT_ERR((((uint32_t)anonymousadvfilten << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)anonymousadvfilten << 10));
}

__INLINE uint8_t ble_rwblecntl_advertfilt_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ble_rwblecntl_advertfilt_en_setf(uint8_t advertfilten)
{
    ASSERT_ERR((((uint32_t)advertfilten << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)advertfilten << 9));
}

__INLINE uint8_t ble_rwblecntl_rwble_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ble_rwblecntl_rwble_en_setf(uint8_t rwbleen)
{
    ASSERT_ERR((((uint32_t)rwbleen << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rwbleen << 8));
}

__INLINE uint8_t ble_rwblecntl_rxwinszdef_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ble_rwblecntl_rxwinszdef_setf(uint8_t rxwinszdef)
{
    ASSERT_ERR((((uint32_t)rxwinszdef << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, (REG_BLE_RD(BLE_RWBLECNTL_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)rxwinszdef << 0));
}

/**
 * @brief VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24                  TYP   0x9
 *  23:16                  REL   0x0
 *  15:08                  UPG   0x1A
 *  07:00                BUILD   0x0
 * </pre>
 */
#define BLE_VERSION_ADDR   0x40100004
#define BLE_VERSION_OFFSET 0x00000004
#define BLE_VERSION_INDEX  0x00000001
#define BLE_VERSION_RESET  0x09001A00

__INLINE uint32_t ble_version_get(void)
{
    return REG_BLE_RD(BLE_VERSION_ADDR);
}

// field definitions
#define BLE_TYP_MASK     ((uint32_t)0xFF000000)
#define BLE_TYP_LSB      24
#define BLE_TYP_WIDTH    ((uint32_t)0x00000008)
#define BLE_REL_MASK     ((uint32_t)0x00FF0000)
#define BLE_REL_LSB      16
#define BLE_REL_WIDTH    ((uint32_t)0x00000008)
#define BLE_UPG_MASK     ((uint32_t)0x0000FF00)
#define BLE_UPG_LSB      8
#define BLE_UPG_WIDTH    ((uint32_t)0x00000008)
#define BLE_BUILD_MASK   ((uint32_t)0x000000FF)
#define BLE_BUILD_LSB    0
#define BLE_BUILD_WIDTH  ((uint32_t)0x00000008)

#define BLE_TYP_RST      0x9
#define BLE_REL_RST      0x0
#define BLE_UPG_RST      0x1A
#define BLE_BUILD_RST    0x0

__INLINE void ble_version_unpack(uint8_t *typ, uint8_t *rel, uint8_t *upg, uint8_t *build)
{
    uint32_t localVal = REG_BLE_RD(BLE_VERSION_ADDR);

    *typ = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rel = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *upg = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *build = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_version_typ_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_VERSION_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t ble_version_rel_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t ble_version_upg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t ble_version_build_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief RWBLECONF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31               DMMODE   0
 *     28           CORRELATOR   1
 *     27              USERXLR   1
 *     26              USETXLR   1
 *  25:24            ISOPORTNB   0x3
 *     21             WLANCOEX   1
 *  20:16                 RFIF   0x1
 *     15               USEDBG   1
 *     14             DECIPHER   0
 *  13:08              CLK_SEL   0x8
 *     07              INTMODE   1
 *     06             BUS_TYPE   0
 *     05           DATA_WIDTH   0
 *  04:00           ADDR_WIDTH   0xD
 * </pre>
 */
#define BLE_RWBLECONF_ADDR   0x40100008
#define BLE_RWBLECONF_OFFSET 0x00000008
#define BLE_RWBLECONF_INDEX  0x00000002
#define BLE_RWBLECONF_RESET  0x1F21888D

__INLINE uint32_t ble_rwbleconf_get(void)
{
    return REG_BLE_RD(BLE_RWBLECONF_ADDR);
}

// field definitions
#define BLE_DMMODE_BIT        ((uint32_t)0x80000000)
#define BLE_DMMODE_POS        31
#define BLE_CORRELATOR_BIT    ((uint32_t)0x10000000)
#define BLE_CORRELATOR_POS    28
#define BLE_USERXLR_BIT       ((uint32_t)0x08000000)
#define BLE_USERXLR_POS       27
#define BLE_USETXLR_BIT       ((uint32_t)0x04000000)
#define BLE_USETXLR_POS       26
#define BLE_ISOPORTNB_MASK    ((uint32_t)0x03000000)
#define BLE_ISOPORTNB_LSB     24
#define BLE_ISOPORTNB_WIDTH   ((uint32_t)0x00000002)
#define BLE_WLANCOEX_BIT      ((uint32_t)0x00200000)
#define BLE_WLANCOEX_POS      21
#define BLE_RFIF_MASK         ((uint32_t)0x001F0000)
#define BLE_RFIF_LSB          16
#define BLE_RFIF_WIDTH        ((uint32_t)0x00000005)
#define BLE_USEDBG_BIT        ((uint32_t)0x00008000)
#define BLE_USEDBG_POS        15
#define BLE_DECIPHER_BIT      ((uint32_t)0x00004000)
#define BLE_DECIPHER_POS      14
#define BLE_CLK_SEL_MASK      ((uint32_t)0x00003F00)
#define BLE_CLK_SEL_LSB       8
#define BLE_CLK_SEL_WIDTH     ((uint32_t)0x00000006)
#define BLE_INTMODE_BIT       ((uint32_t)0x00000080)
#define BLE_INTMODE_POS       7
#define BLE_BUS_TYPE_BIT      ((uint32_t)0x00000040)
#define BLE_BUS_TYPE_POS      6
#define BLE_DATA_WIDTH_BIT    ((uint32_t)0x00000020)
#define BLE_DATA_WIDTH_POS    5
#define BLE_ADDR_WIDTH_MASK   ((uint32_t)0x0000001F)
#define BLE_ADDR_WIDTH_LSB    0
#define BLE_ADDR_WIDTH_WIDTH  ((uint32_t)0x00000005)

#define BLE_DMMODE_RST        0x0
#define BLE_CORRELATOR_RST    0x1
#define BLE_USERXLR_RST       0x1
#define BLE_USETXLR_RST       0x1
#define BLE_ISOPORTNB_RST     0x3
#define BLE_WLANCOEX_RST      0x1
#define BLE_RFIF_RST          0x1
#define BLE_USEDBG_RST        0x1
#define BLE_DECIPHER_RST      0x0
#define BLE_CLK_SEL_RST       0x8
#define BLE_INTMODE_RST       0x1
#define BLE_BUS_TYPE_RST      0x0
#define BLE_DATA_WIDTH_RST    0x0
#define BLE_ADDR_WIDTH_RST    0xD

__INLINE void ble_rwbleconf_unpack(uint8_t *dmmode, uint8_t *correlator, uint8_t *userxlr, uint8_t *usetxlr, uint8_t *isoportnb, uint8_t *wlancoex, uint8_t *rfif, uint8_t *usedbg, uint8_t *decipher, uint8_t *clksel, uint8_t *intmode, uint8_t *bustype, uint8_t *datawidth, uint8_t *addrwidth)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);

    *dmmode = (localVal & ((uint32_t)0x80000000)) >> 31;
    *correlator = (localVal & ((uint32_t)0x10000000)) >> 28;
    *userxlr = (localVal & ((uint32_t)0x08000000)) >> 27;
    *usetxlr = (localVal & ((uint32_t)0x04000000)) >> 26;
    *isoportnb = (localVal & ((uint32_t)0x03000000)) >> 24;
    *wlancoex = (localVal & ((uint32_t)0x00200000)) >> 21;
    *rfif = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *usedbg = (localVal & ((uint32_t)0x00008000)) >> 15;
    *decipher = (localVal & ((uint32_t)0x00004000)) >> 14;
    *clksel = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *intmode = (localVal & ((uint32_t)0x00000080)) >> 7;
    *bustype = (localVal & ((uint32_t)0x00000040)) >> 6;
    *datawidth = (localVal & ((uint32_t)0x00000020)) >> 5;
    *addrwidth = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t ble_rwbleconf_dmmode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t ble_rwbleconf_correlator_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE uint8_t ble_rwbleconf_userxlr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE uint8_t ble_rwbleconf_usetxlr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE uint8_t ble_rwbleconf_isoportnb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE uint8_t ble_rwbleconf_wlancoex_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE uint8_t ble_rwbleconf_rfif_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE uint8_t ble_rwbleconf_usedbg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t ble_rwbleconf_decipher_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t ble_rwbleconf_clk_sel_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE uint8_t ble_rwbleconf_intmode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t ble_rwbleconf_bus_type_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t ble_rwbleconf_data_width_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t ble_rwbleconf_addr_width_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RWBLECONF_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

/**
 * @brief INTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:28         CLKNINTSRMSK   0x0
 *  27:24         CLKNINTSRVAL   0x0
 *     15         AUDIOINT2MSK   0
 *     14         AUDIOINT1MSK   0
 *     13         AUDIOINT0MSK   0
 *     12             SWINTMSK   0
 *     11   TIMESTAMPTGTINTMSK   0
 *     10      FINETGTIMINTMSK   0
 *     09     GROSSTGTIMINTMSK   0
 *     08          ERRORINTMSK   0
 *     07          CRYPTINTMSK   0
 *     06        SKIPEVTINTMSK   0
 *     05         ENDEVTINTMSK   1
 *     04       STARTEVTINTMSK   1
 *     03            SLPINTMSK   1
 *     02             RXINTMSK   1
 *     01             TXINTMSK   1
 *     00           CLKNINTMSK   1
 * </pre>
 */
#define BLE_INTCNTL_ADDR   0x4010000C
#define BLE_INTCNTL_OFFSET 0x0000000C
#define BLE_INTCNTL_INDEX  0x00000003
#define BLE_INTCNTL_RESET  0x0000003F

__INLINE uint32_t ble_intcntl_get(void)
{
    return REG_BLE_RD(BLE_INTCNTL_ADDR);
}

__INLINE void ble_intcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_INTCNTL_ADDR, value);
}

// field definitions
#define BLE_CLKNINTSRMSK_MASK         ((uint32_t)0x70000000)
#define BLE_CLKNINTSRMSK_LSB          28
#define BLE_CLKNINTSRMSK_WIDTH        ((uint32_t)0x00000003)
#define BLE_CLKNINTSRVAL_MASK         ((uint32_t)0x0F000000)
#define BLE_CLKNINTSRVAL_LSB          24
#define BLE_CLKNINTSRVAL_WIDTH        ((uint32_t)0x00000004)
#define BLE_AUDIOINT2MSK_BIT          ((uint32_t)0x00008000)
#define BLE_AUDIOINT2MSK_POS          15
#define BLE_AUDIOINT1MSK_BIT          ((uint32_t)0x00004000)
#define BLE_AUDIOINT1MSK_POS          14
#define BLE_AUDIOINT0MSK_BIT          ((uint32_t)0x00002000)
#define BLE_AUDIOINT0MSK_POS          13
#define BLE_SWINTMSK_BIT              ((uint32_t)0x00001000)
#define BLE_SWINTMSK_POS              12
#define BLE_TIMESTAMPTGTINTMSK_BIT    ((uint32_t)0x00000800)
#define BLE_TIMESTAMPTGTINTMSK_POS    11
#define BLE_FINETGTIMINTMSK_BIT       ((uint32_t)0x00000400)
#define BLE_FINETGTIMINTMSK_POS       10
#define BLE_GROSSTGTIMINTMSK_BIT      ((uint32_t)0x00000200)
#define BLE_GROSSTGTIMINTMSK_POS      9
#define BLE_ERRORINTMSK_BIT           ((uint32_t)0x00000100)
#define BLE_ERRORINTMSK_POS           8
#define BLE_CRYPTINTMSK_BIT           ((uint32_t)0x00000080)
#define BLE_CRYPTINTMSK_POS           7
#define BLE_SKIPEVTINTMSK_BIT         ((uint32_t)0x00000040)
#define BLE_SKIPEVTINTMSK_POS         6
#define BLE_ENDEVTINTMSK_BIT          ((uint32_t)0x00000020)
#define BLE_ENDEVTINTMSK_POS          5
#define BLE_STARTEVTINTMSK_BIT        ((uint32_t)0x00000010)
#define BLE_STARTEVTINTMSK_POS        4
#define BLE_SLPINTMSK_BIT             ((uint32_t)0x00000008)
#define BLE_SLPINTMSK_POS             3
#define BLE_RXINTMSK_BIT              ((uint32_t)0x00000004)
#define BLE_RXINTMSK_POS              2
#define BLE_TXINTMSK_BIT              ((uint32_t)0x00000002)
#define BLE_TXINTMSK_POS              1
#define BLE_CLKNINTMSK_BIT            ((uint32_t)0x00000001)
#define BLE_CLKNINTMSK_POS            0

#define BLE_CLKNINTSRMSK_RST          0x0
#define BLE_CLKNINTSRVAL_RST          0x0
#define BLE_AUDIOINT2MSK_RST          0x0
#define BLE_AUDIOINT1MSK_RST          0x0
#define BLE_AUDIOINT0MSK_RST          0x0
#define BLE_SWINTMSK_RST              0x0
#define BLE_TIMESTAMPTGTINTMSK_RST    0x0
#define BLE_FINETGTIMINTMSK_RST       0x0
#define BLE_GROSSTGTIMINTMSK_RST      0x0
#define BLE_ERRORINTMSK_RST           0x0
#define BLE_CRYPTINTMSK_RST           0x0
#define BLE_SKIPEVTINTMSK_RST         0x0
#define BLE_ENDEVTINTMSK_RST          0x1
#define BLE_STARTEVTINTMSK_RST        0x1
#define BLE_SLPINTMSK_RST             0x1
#define BLE_RXINTMSK_RST              0x1
#define BLE_TXINTMSK_RST              0x1
#define BLE_CLKNINTMSK_RST            0x1

__INLINE void ble_intcntl_pack(uint8_t clknintsrmsk, uint8_t clknintsrval, uint8_t audioint2msk, uint8_t audioint1msk, uint8_t audioint0msk, uint8_t swintmsk, uint8_t timestamptgtintmsk, uint8_t finetgtimintmsk, uint8_t grosstgtimintmsk, uint8_t errorintmsk, uint8_t cryptintmsk, uint8_t skipevtintmsk, uint8_t endevtintmsk, uint8_t startevtintmsk, uint8_t slpintmsk, uint8_t rxintmsk, uint8_t txintmsk, uint8_t clknintmsk)
{
    ASSERT_ERR((((uint32_t)clknintsrmsk << 28) & ~((uint32_t)0x70000000)) == 0);
    ASSERT_ERR((((uint32_t)clknintsrval << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)audioint2msk << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)audioint1msk << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)audioint0msk << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)swintmsk << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgtintmsk << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)finetgtimintmsk << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)grosstgtimintmsk << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)errorintmsk << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)cryptintmsk << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)skipevtintmsk << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)endevtintmsk << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)startevtintmsk << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)slpintmsk << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)rxintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)txintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)clknintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, ((uint32_t)clknintsrmsk << 28) | ((uint32_t)clknintsrval << 24) | ((uint32_t)audioint2msk << 15) | ((uint32_t)audioint1msk << 14) | ((uint32_t)audioint0msk << 13) | ((uint32_t)swintmsk << 12) | ((uint32_t)timestamptgtintmsk << 11) | ((uint32_t)finetgtimintmsk << 10) | ((uint32_t)grosstgtimintmsk << 9) | ((uint32_t)errorintmsk << 8) | ((uint32_t)cryptintmsk << 7) | ((uint32_t)skipevtintmsk << 6) | ((uint32_t)endevtintmsk << 5) | ((uint32_t)startevtintmsk << 4) | ((uint32_t)slpintmsk << 3) | ((uint32_t)rxintmsk << 2) | ((uint32_t)txintmsk << 1) | ((uint32_t)clknintmsk << 0));
}

__INLINE void ble_intcntl_unpack(uint8_t *clknintsrmsk, uint8_t *clknintsrval, uint8_t *audioint2msk, uint8_t *audioint1msk, uint8_t *audioint0msk, uint8_t *swintmsk, uint8_t *timestamptgtintmsk, uint8_t *finetgtimintmsk, uint8_t *grosstgtimintmsk, uint8_t *errorintmsk, uint8_t *cryptintmsk, uint8_t *skipevtintmsk, uint8_t *endevtintmsk, uint8_t *startevtintmsk, uint8_t *slpintmsk, uint8_t *rxintmsk, uint8_t *txintmsk, uint8_t *clknintmsk)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);

    *clknintsrmsk = (localVal & ((uint32_t)0x70000000)) >> 28;
    *clknintsrval = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *audioint2msk = (localVal & ((uint32_t)0x00008000)) >> 15;
    *audioint1msk = (localVal & ((uint32_t)0x00004000)) >> 14;
    *audioint0msk = (localVal & ((uint32_t)0x00002000)) >> 13;
    *swintmsk = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestamptgtintmsk = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtimintmsk = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtimintmsk = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintmsk = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintmsk = (localVal & ((uint32_t)0x00000080)) >> 7;
    *skipevtintmsk = (localVal & ((uint32_t)0x00000040)) >> 6;
    *endevtintmsk = (localVal & ((uint32_t)0x00000020)) >> 5;
    *startevtintmsk = (localVal & ((uint32_t)0x00000010)) >> 4;
    *slpintmsk = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxintmsk = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txintmsk = (localVal & ((uint32_t)0x00000002)) >> 1;
    *clknintmsk = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_intcntl_clknintsrmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x70000000)) >> 28);
}

__INLINE void ble_intcntl_clknintsrmsk_setf(uint8_t clknintsrmsk)
{
    ASSERT_ERR((((uint32_t)clknintsrmsk << 28) & ~((uint32_t)0x70000000)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x70000000)) | ((uint32_t)clknintsrmsk << 28));
}

__INLINE uint8_t ble_intcntl_clknintsrval_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void ble_intcntl_clknintsrval_setf(uint8_t clknintsrval)
{
    ASSERT_ERR((((uint32_t)clknintsrval << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)clknintsrval << 24));
}

__INLINE uint8_t ble_intcntl_audioint2msk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ble_intcntl_audioint2msk_setf(uint8_t audioint2msk)
{
    ASSERT_ERR((((uint32_t)audioint2msk << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)audioint2msk << 15));
}

__INLINE uint8_t ble_intcntl_audioint1msk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ble_intcntl_audioint1msk_setf(uint8_t audioint1msk)
{
    ASSERT_ERR((((uint32_t)audioint1msk << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)audioint1msk << 14));
}

__INLINE uint8_t ble_intcntl_audioint0msk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ble_intcntl_audioint0msk_setf(uint8_t audioint0msk)
{
    ASSERT_ERR((((uint32_t)audioint0msk << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)audioint0msk << 13));
}

__INLINE uint8_t ble_intcntl_swintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ble_intcntl_swintmsk_setf(uint8_t swintmsk)
{
    ASSERT_ERR((((uint32_t)swintmsk << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)swintmsk << 12));
}

__INLINE uint8_t ble_intcntl_timestamptgtintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ble_intcntl_timestamptgtintmsk_setf(uint8_t timestamptgtintmsk)
{
    ASSERT_ERR((((uint32_t)timestamptgtintmsk << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)timestamptgtintmsk << 11));
}

__INLINE uint8_t ble_intcntl_finetgtimintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ble_intcntl_finetgtimintmsk_setf(uint8_t finetgtimintmsk)
{
    ASSERT_ERR((((uint32_t)finetgtimintmsk << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)finetgtimintmsk << 10));
}

__INLINE uint8_t ble_intcntl_grosstgtimintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ble_intcntl_grosstgtimintmsk_setf(uint8_t grosstgtimintmsk)
{
    ASSERT_ERR((((uint32_t)grosstgtimintmsk << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)grosstgtimintmsk << 9));
}

__INLINE uint8_t ble_intcntl_errorintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ble_intcntl_errorintmsk_setf(uint8_t errorintmsk)
{
    ASSERT_ERR((((uint32_t)errorintmsk << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)errorintmsk << 8));
}

__INLINE uint8_t ble_intcntl_cryptintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ble_intcntl_cryptintmsk_setf(uint8_t cryptintmsk)
{
    ASSERT_ERR((((uint32_t)cryptintmsk << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)cryptintmsk << 7));
}

__INLINE uint8_t ble_intcntl_skipevtintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void ble_intcntl_skipevtintmsk_setf(uint8_t skipevtintmsk)
{
    ASSERT_ERR((((uint32_t)skipevtintmsk << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)skipevtintmsk << 6));
}

__INLINE uint8_t ble_intcntl_endevtintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void ble_intcntl_endevtintmsk_setf(uint8_t endevtintmsk)
{
    ASSERT_ERR((((uint32_t)endevtintmsk << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)endevtintmsk << 5));
}

__INLINE uint8_t ble_intcntl_startevtintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ble_intcntl_startevtintmsk_setf(uint8_t startevtintmsk)
{
    ASSERT_ERR((((uint32_t)startevtintmsk << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)startevtintmsk << 4));
}

__INLINE uint8_t ble_intcntl_slpintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_intcntl_slpintmsk_setf(uint8_t slpintmsk)
{
    ASSERT_ERR((((uint32_t)slpintmsk << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)slpintmsk << 3));
}

__INLINE uint8_t ble_intcntl_rxintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_intcntl_rxintmsk_setf(uint8_t rxintmsk)
{
    ASSERT_ERR((((uint32_t)rxintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rxintmsk << 2));
}

__INLINE uint8_t ble_intcntl_txintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ble_intcntl_txintmsk_setf(uint8_t txintmsk)
{
    ASSERT_ERR((((uint32_t)txintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)txintmsk << 1));
}

__INLINE uint8_t ble_intcntl_clknintmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_intcntl_clknintmsk_setf(uint8_t clknintmsk)
{
    ASSERT_ERR((((uint32_t)clknintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_INTCNTL_ADDR, (REG_BLE_RD(BLE_INTCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)clknintmsk << 0));
}

/**
 * @brief INTSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15        AUDIOINT2STAT   0
 *     14        AUDIOINT1STAT   0
 *     13        AUDIOINT0STAT   0
 *     12            SWINTSTAT   0
 *     11   TIMESTAMPTGTINTSTAT   0
 *     10     FINETGTIMINTSTAT   0
 *     09    GROSSTGTIMINTSTAT   0
 *     08         ERRORINTSTAT   0
 *     07         CRYPTINTSTAT   0
 *     06       SKIPEVTINTSTAT   0
 *     05        ENDEVTINTSTAT   0
 *     04      STARTEVTINTSTAT   0
 *     03           SLPINTSTAT   0
 *     02            RXINTSTAT   0
 *     01            TXINTSTAT   0
 *     00          CLKNINTSTAT   0
 * </pre>
 */
#define BLE_INTSTAT_ADDR   0x40100010
#define BLE_INTSTAT_OFFSET 0x00000010
#define BLE_INTSTAT_INDEX  0x00000004
#define BLE_INTSTAT_RESET  0x00000000

__INLINE uint32_t ble_intstat_get(void)
{
    return REG_BLE_RD(BLE_INTSTAT_ADDR);
}

// field definitions
#define BLE_AUDIOINT2STAT_BIT          ((uint32_t)0x00008000)
#define BLE_AUDIOINT2STAT_POS          15
#define BLE_AUDIOINT1STAT_BIT          ((uint32_t)0x00004000)
#define BLE_AUDIOINT1STAT_POS          14
#define BLE_AUDIOINT0STAT_BIT          ((uint32_t)0x00002000)
#define BLE_AUDIOINT0STAT_POS          13
#define BLE_SWINTSTAT_BIT              ((uint32_t)0x00001000)
#define BLE_SWINTSTAT_POS              12
#define BLE_TIMESTAMPTGTINTSTAT_BIT    ((uint32_t)0x00000800)
#define BLE_TIMESTAMPTGTINTSTAT_POS    11
#define BLE_FINETGTIMINTSTAT_BIT       ((uint32_t)0x00000400)
#define BLE_FINETGTIMINTSTAT_POS       10
#define BLE_GROSSTGTIMINTSTAT_BIT      ((uint32_t)0x00000200)
#define BLE_GROSSTGTIMINTSTAT_POS      9
#define BLE_ERRORINTSTAT_BIT           ((uint32_t)0x00000100)
#define BLE_ERRORINTSTAT_POS           8
#define BLE_CRYPTINTSTAT_BIT           ((uint32_t)0x00000080)
#define BLE_CRYPTINTSTAT_POS           7
#define BLE_SKIPEVTINTSTAT_BIT         ((uint32_t)0x00000040)
#define BLE_SKIPEVTINTSTAT_POS         6
#define BLE_ENDEVTINTSTAT_BIT          ((uint32_t)0x00000020)
#define BLE_ENDEVTINTSTAT_POS          5
#define BLE_STARTEVTINTSTAT_BIT        ((uint32_t)0x00000010)
#define BLE_STARTEVTINTSTAT_POS        4
#define BLE_SLPINTSTAT_BIT             ((uint32_t)0x00000008)
#define BLE_SLPINTSTAT_POS             3
#define BLE_RXINTSTAT_BIT              ((uint32_t)0x00000004)
#define BLE_RXINTSTAT_POS              2
#define BLE_TXINTSTAT_BIT              ((uint32_t)0x00000002)
#define BLE_TXINTSTAT_POS              1
#define BLE_CLKNINTSTAT_BIT            ((uint32_t)0x00000001)
#define BLE_CLKNINTSTAT_POS            0

#define BLE_AUDIOINT2STAT_RST          0x0
#define BLE_AUDIOINT1STAT_RST          0x0
#define BLE_AUDIOINT0STAT_RST          0x0
#define BLE_SWINTSTAT_RST              0x0
#define BLE_TIMESTAMPTGTINTSTAT_RST    0x0
#define BLE_FINETGTIMINTSTAT_RST       0x0
#define BLE_GROSSTGTIMINTSTAT_RST      0x0
#define BLE_ERRORINTSTAT_RST           0x0
#define BLE_CRYPTINTSTAT_RST           0x0
#define BLE_SKIPEVTINTSTAT_RST         0x0
#define BLE_ENDEVTINTSTAT_RST          0x0
#define BLE_STARTEVTINTSTAT_RST        0x0
#define BLE_SLPINTSTAT_RST             0x0
#define BLE_RXINTSTAT_RST              0x0
#define BLE_TXINTSTAT_RST              0x0
#define BLE_CLKNINTSTAT_RST            0x0

__INLINE void ble_intstat_unpack(uint8_t *audioint2stat, uint8_t *audioint1stat, uint8_t *audioint0stat, uint8_t *swintstat, uint8_t *timestamptgtintstat, uint8_t *finetgtimintstat, uint8_t *grosstgtimintstat, uint8_t *errorintstat, uint8_t *cryptintstat, uint8_t *skipevtintstat, uint8_t *endevtintstat, uint8_t *startevtintstat, uint8_t *slpintstat, uint8_t *rxintstat, uint8_t *txintstat, uint8_t *clknintstat)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);

    *audioint2stat = (localVal & ((uint32_t)0x00008000)) >> 15;
    *audioint1stat = (localVal & ((uint32_t)0x00004000)) >> 14;
    *audioint0stat = (localVal & ((uint32_t)0x00002000)) >> 13;
    *swintstat = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestamptgtintstat = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtimintstat = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtimintstat = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintstat = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintstat = (localVal & ((uint32_t)0x00000080)) >> 7;
    *skipevtintstat = (localVal & ((uint32_t)0x00000040)) >> 6;
    *endevtintstat = (localVal & ((uint32_t)0x00000020)) >> 5;
    *startevtintstat = (localVal & ((uint32_t)0x00000010)) >> 4;
    *slpintstat = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxintstat = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txintstat = (localVal & ((uint32_t)0x00000002)) >> 1;
    *clknintstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_intstat_audioint2stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t ble_intstat_audioint1stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t ble_intstat_audioint0stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t ble_intstat_swintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t ble_intstat_timestamptgtintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t ble_intstat_finetgtimintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t ble_intstat_grosstgtimintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t ble_intstat_errorintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t ble_intstat_cryptintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t ble_intstat_skipevtintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t ble_intstat_endevtintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t ble_intstat_startevtintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t ble_intstat_slpintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t ble_intstat_rxintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ble_intstat_txintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ble_intstat_clknintstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief INTRAWSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15     AUDIOINT2RAWSTAT   0
 *     14     AUDIOINT1RAWSTAT   0
 *     13     AUDIOINT0RAWSTAT   0
 *     12         SWINTRAWSTAT   0
 *     11   TIMESTAMPTGTINTRAWSTAT   0
 *     10   FINETGTIMINTRAWSTAT   0
 *     09   GROSSTGTIMINTRAWSTAT   0
 *     08      ERRORINTRAWSTAT   0
 *     07      CRYPTINTRAWSTAT   0
 *     06    SKIPEVTINTRAWSTAT   0
 *     05     ENDEVTINTRAWSTAT   0
 *     04   STARTEVTINTRAWSTAT   0
 *     03        SLPINTRAWSTAT   0
 *     02         RXINTRAWSTAT   0
 *     01         TXINTRAWSTAT   0
 *     00       CLKNINTRAWSTAT   0
 * </pre>
 */
#define BLE_INTRAWSTAT_ADDR   0x40100014
#define BLE_INTRAWSTAT_OFFSET 0x00000014
#define BLE_INTRAWSTAT_INDEX  0x00000005
#define BLE_INTRAWSTAT_RESET  0x00000000

__INLINE uint32_t ble_intrawstat_get(void)
{
    return REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
}

// field definitions
#define BLE_AUDIOINT2RAWSTAT_BIT          ((uint32_t)0x00008000)
#define BLE_AUDIOINT2RAWSTAT_POS          15
#define BLE_AUDIOINT1RAWSTAT_BIT          ((uint32_t)0x00004000)
#define BLE_AUDIOINT1RAWSTAT_POS          14
#define BLE_AUDIOINT0RAWSTAT_BIT          ((uint32_t)0x00002000)
#define BLE_AUDIOINT0RAWSTAT_POS          13
#define BLE_SWINTRAWSTAT_BIT              ((uint32_t)0x00001000)
#define BLE_SWINTRAWSTAT_POS              12
#define BLE_TIMESTAMPTGTINTRAWSTAT_BIT    ((uint32_t)0x00000800)
#define BLE_TIMESTAMPTGTINTRAWSTAT_POS    11
#define BLE_FINETGTIMINTRAWSTAT_BIT       ((uint32_t)0x00000400)
#define BLE_FINETGTIMINTRAWSTAT_POS       10
#define BLE_GROSSTGTIMINTRAWSTAT_BIT      ((uint32_t)0x00000200)
#define BLE_GROSSTGTIMINTRAWSTAT_POS      9
#define BLE_ERRORINTRAWSTAT_BIT           ((uint32_t)0x00000100)
#define BLE_ERRORINTRAWSTAT_POS           8
#define BLE_CRYPTINTRAWSTAT_BIT           ((uint32_t)0x00000080)
#define BLE_CRYPTINTRAWSTAT_POS           7
#define BLE_SKIPEVTINTRAWSTAT_BIT         ((uint32_t)0x00000040)
#define BLE_SKIPEVTINTRAWSTAT_POS         6
#define BLE_ENDEVTINTRAWSTAT_BIT          ((uint32_t)0x00000020)
#define BLE_ENDEVTINTRAWSTAT_POS          5
#define BLE_STARTEVTINTRAWSTAT_BIT        ((uint32_t)0x00000010)
#define BLE_STARTEVTINTRAWSTAT_POS        4
#define BLE_SLPINTRAWSTAT_BIT             ((uint32_t)0x00000008)
#define BLE_SLPINTRAWSTAT_POS             3
#define BLE_RXINTRAWSTAT_BIT              ((uint32_t)0x00000004)
#define BLE_RXINTRAWSTAT_POS              2
#define BLE_TXINTRAWSTAT_BIT              ((uint32_t)0x00000002)
#define BLE_TXINTRAWSTAT_POS              1
#define BLE_CLKNINTRAWSTAT_BIT            ((uint32_t)0x00000001)
#define BLE_CLKNINTRAWSTAT_POS            0

#define BLE_AUDIOINT2RAWSTAT_RST          0x0
#define BLE_AUDIOINT1RAWSTAT_RST          0x0
#define BLE_AUDIOINT0RAWSTAT_RST          0x0
#define BLE_SWINTRAWSTAT_RST              0x0
#define BLE_TIMESTAMPTGTINTRAWSTAT_RST    0x0
#define BLE_FINETGTIMINTRAWSTAT_RST       0x0
#define BLE_GROSSTGTIMINTRAWSTAT_RST      0x0
#define BLE_ERRORINTRAWSTAT_RST           0x0
#define BLE_CRYPTINTRAWSTAT_RST           0x0
#define BLE_SKIPEVTINTRAWSTAT_RST         0x0
#define BLE_ENDEVTINTRAWSTAT_RST          0x0
#define BLE_STARTEVTINTRAWSTAT_RST        0x0
#define BLE_SLPINTRAWSTAT_RST             0x0
#define BLE_RXINTRAWSTAT_RST              0x0
#define BLE_TXINTRAWSTAT_RST              0x0
#define BLE_CLKNINTRAWSTAT_RST            0x0

__INLINE void ble_intrawstat_unpack(uint8_t *audioint2rawstat, uint8_t *audioint1rawstat, uint8_t *audioint0rawstat, uint8_t *swintrawstat, uint8_t *timestamptgtintrawstat, uint8_t *finetgtimintrawstat, uint8_t *grosstgtimintrawstat, uint8_t *errorintrawstat, uint8_t *cryptintrawstat, uint8_t *skipevtintrawstat, uint8_t *endevtintrawstat, uint8_t *startevtintrawstat, uint8_t *slpintrawstat, uint8_t *rxintrawstat, uint8_t *txintrawstat, uint8_t *clknintrawstat)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);

    *audioint2rawstat = (localVal & ((uint32_t)0x00008000)) >> 15;
    *audioint1rawstat = (localVal & ((uint32_t)0x00004000)) >> 14;
    *audioint0rawstat = (localVal & ((uint32_t)0x00002000)) >> 13;
    *swintrawstat = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestamptgtintrawstat = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtimintrawstat = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtimintrawstat = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintrawstat = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintrawstat = (localVal & ((uint32_t)0x00000080)) >> 7;
    *skipevtintrawstat = (localVal & ((uint32_t)0x00000040)) >> 6;
    *endevtintrawstat = (localVal & ((uint32_t)0x00000020)) >> 5;
    *startevtintrawstat = (localVal & ((uint32_t)0x00000010)) >> 4;
    *slpintrawstat = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxintrawstat = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txintrawstat = (localVal & ((uint32_t)0x00000002)) >> 1;
    *clknintrawstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_intrawstat_audioint2rawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t ble_intrawstat_audioint1rawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t ble_intrawstat_audioint0rawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t ble_intrawstat_swintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t ble_intrawstat_timestamptgtintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t ble_intrawstat_finetgtimintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t ble_intrawstat_grosstgtimintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t ble_intrawstat_errorintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t ble_intrawstat_cryptintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t ble_intrawstat_skipevtintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t ble_intrawstat_endevtintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t ble_intrawstat_startevtintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t ble_intrawstat_slpintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t ble_intrawstat_rxintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ble_intrawstat_txintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ble_intrawstat_clknintrawstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief INTACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15         AUDIOINT2ACK   0
 *     14         AUDIOINT1ACK   0
 *     13         AUDIOINT0ACK   0
 *     12             SWINTACK   0
 *     11   TIMESTAMPTGTINTACK   0
 *     10      FINETGTIMINTACK   0
 *     09     GROSSTGTIMINTACK   0
 *     08          ERRORINTACK   0
 *     07          CRYPTINTACK   0
 *     06        SKIPEVTINTACK   0
 *     05         ENDEVTINTACK   0
 *     04       STARTEVTINTACK   0
 *     03            SLPINTACK   0
 *     02             RXINTACK   0
 *     01             TXINTACK   0
 *     00           CLKNINTACK   0
 * </pre>
 */
#define BLE_INTACK_ADDR   0x40100018
#define BLE_INTACK_OFFSET 0x00000018
#define BLE_INTACK_INDEX  0x00000006
#define BLE_INTACK_RESET  0x00000000

__INLINE uint32_t ble_intack_get(void)
{
    return REG_BLE_RD(BLE_INTACK_ADDR);
}

__INLINE void ble_intack_clear(uint32_t value)
{
    REG_BLE_WR(BLE_INTACK_ADDR, value);
}

// field definitions
#define BLE_AUDIOINT2ACK_BIT          ((uint32_t)0x00008000)
#define BLE_AUDIOINT2ACK_POS          15
#define BLE_AUDIOINT1ACK_BIT          ((uint32_t)0x00004000)
#define BLE_AUDIOINT1ACK_POS          14
#define BLE_AUDIOINT0ACK_BIT          ((uint32_t)0x00002000)
#define BLE_AUDIOINT0ACK_POS          13
#define BLE_SWINTACK_BIT              ((uint32_t)0x00001000)
#define BLE_SWINTACK_POS              12
#define BLE_TIMESTAMPTGTINTACK_BIT    ((uint32_t)0x00000800)
#define BLE_TIMESTAMPTGTINTACK_POS    11
#define BLE_FINETGTIMINTACK_BIT       ((uint32_t)0x00000400)
#define BLE_FINETGTIMINTACK_POS       10
#define BLE_GROSSTGTIMINTACK_BIT      ((uint32_t)0x00000200)
#define BLE_GROSSTGTIMINTACK_POS      9
#define BLE_ERRORINTACK_BIT           ((uint32_t)0x00000100)
#define BLE_ERRORINTACK_POS           8
#define BLE_CRYPTINTACK_BIT           ((uint32_t)0x00000080)
#define BLE_CRYPTINTACK_POS           7
#define BLE_SKIPEVTINTACK_BIT         ((uint32_t)0x00000040)
#define BLE_SKIPEVTINTACK_POS         6
#define BLE_ENDEVTINTACK_BIT          ((uint32_t)0x00000020)
#define BLE_ENDEVTINTACK_POS          5
#define BLE_STARTEVTINTACK_BIT        ((uint32_t)0x00000010)
#define BLE_STARTEVTINTACK_POS        4
#define BLE_SLPINTACK_BIT             ((uint32_t)0x00000008)
#define BLE_SLPINTACK_POS             3
#define BLE_RXINTACK_BIT              ((uint32_t)0x00000004)
#define BLE_RXINTACK_POS              2
#define BLE_TXINTACK_BIT              ((uint32_t)0x00000002)
#define BLE_TXINTACK_POS              1
#define BLE_CLKNINTACK_BIT            ((uint32_t)0x00000001)
#define BLE_CLKNINTACK_POS            0

#define BLE_AUDIOINT2ACK_RST          0x0
#define BLE_AUDIOINT1ACK_RST          0x0
#define BLE_AUDIOINT0ACK_RST          0x0
#define BLE_SWINTACK_RST              0x0
#define BLE_TIMESTAMPTGTINTACK_RST    0x0
#define BLE_FINETGTIMINTACK_RST       0x0
#define BLE_GROSSTGTIMINTACK_RST      0x0
#define BLE_ERRORINTACK_RST           0x0
#define BLE_CRYPTINTACK_RST           0x0
#define BLE_SKIPEVTINTACK_RST         0x0
#define BLE_ENDEVTINTACK_RST          0x0
#define BLE_STARTEVTINTACK_RST        0x0
#define BLE_SLPINTACK_RST             0x0
#define BLE_RXINTACK_RST              0x0
#define BLE_TXINTACK_RST              0x0
#define BLE_CLKNINTACK_RST            0x0

__INLINE void ble_intack_pack(uint8_t audioint2ack, uint8_t audioint1ack, uint8_t audioint0ack, uint8_t swintack, uint8_t timestamptgtintack, uint8_t finetgtimintack, uint8_t grosstgtimintack, uint8_t errorintack, uint8_t cryptintack, uint8_t skipevtintack, uint8_t endevtintack, uint8_t startevtintack, uint8_t slpintack, uint8_t rxintack, uint8_t txintack, uint8_t clknintack)
{
    ASSERT_ERR((((uint32_t)audioint2ack << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)audioint1ack << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)audioint0ack << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)swintack << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)timestamptgtintack << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)finetgtimintack << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)grosstgtimintack << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)errorintack << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)cryptintack << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)skipevtintack << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)endevtintack << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)startevtintack << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)slpintack << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)rxintack << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)txintack << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)clknintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, ((uint32_t)audioint2ack << 15) | ((uint32_t)audioint1ack << 14) | ((uint32_t)audioint0ack << 13) | ((uint32_t)swintack << 12) | ((uint32_t)timestamptgtintack << 11) | ((uint32_t)finetgtimintack << 10) | ((uint32_t)grosstgtimintack << 9) | ((uint32_t)errorintack << 8) | ((uint32_t)cryptintack << 7) | ((uint32_t)skipevtintack << 6) | ((uint32_t)endevtintack << 5) | ((uint32_t)startevtintack << 4) | ((uint32_t)slpintack << 3) | ((uint32_t)rxintack << 2) | ((uint32_t)txintack << 1) | ((uint32_t)clknintack << 0));
}

__INLINE void ble_intack_unpack(uint8_t *audioint2ack, uint8_t *audioint1ack, uint8_t *audioint0ack, uint8_t *swintack, uint8_t *timestamptgtintack, uint8_t *finetgtimintack, uint8_t *grosstgtimintack, uint8_t *errorintack, uint8_t *cryptintack, uint8_t *skipevtintack, uint8_t *endevtintack, uint8_t *startevtintack, uint8_t *slpintack, uint8_t *rxintack, uint8_t *txintack, uint8_t *clknintack)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);

    *audioint2ack = (localVal & ((uint32_t)0x00008000)) >> 15;
    *audioint1ack = (localVal & ((uint32_t)0x00004000)) >> 14;
    *audioint0ack = (localVal & ((uint32_t)0x00002000)) >> 13;
    *swintack = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestamptgtintack = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtimintack = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtimintack = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintack = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintack = (localVal & ((uint32_t)0x00000080)) >> 7;
    *skipevtintack = (localVal & ((uint32_t)0x00000040)) >> 6;
    *endevtintack = (localVal & ((uint32_t)0x00000020)) >> 5;
    *startevtintack = (localVal & ((uint32_t)0x00000010)) >> 4;
    *slpintack = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxintack = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txintack = (localVal & ((uint32_t)0x00000002)) >> 1;
    *clknintack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_intack_audioint2ack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ble_intack_audioint2ack_clearf(uint8_t audioint2ack)
{
    ASSERT_ERR((((uint32_t)audioint2ack << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)audioint2ack << 15);
}

__INLINE uint8_t ble_intack_audioint1ack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ble_intack_audioint1ack_clearf(uint8_t audioint1ack)
{
    ASSERT_ERR((((uint32_t)audioint1ack << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)audioint1ack << 14);
}

__INLINE uint8_t ble_intack_audioint0ack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ble_intack_audioint0ack_clearf(uint8_t audioint0ack)
{
    ASSERT_ERR((((uint32_t)audioint0ack << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)audioint0ack << 13);
}

__INLINE uint8_t ble_intack_swintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ble_intack_swintack_clearf(uint8_t swintack)
{
    ASSERT_ERR((((uint32_t)swintack << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)swintack << 12);
}

__INLINE uint8_t ble_intack_timestamptgtintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ble_intack_timestamptgtintack_clearf(uint8_t timestamptgtintack)
{
    ASSERT_ERR((((uint32_t)timestamptgtintack << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)timestamptgtintack << 11);
}

__INLINE uint8_t ble_intack_finetgtimintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ble_intack_finetgtimintack_clearf(uint8_t finetgtimintack)
{
    ASSERT_ERR((((uint32_t)finetgtimintack << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)finetgtimintack << 10);
}

__INLINE uint8_t ble_intack_grosstgtimintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ble_intack_grosstgtimintack_clearf(uint8_t grosstgtimintack)
{
    ASSERT_ERR((((uint32_t)grosstgtimintack << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)grosstgtimintack << 9);
}

__INLINE uint8_t ble_intack_errorintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ble_intack_errorintack_clearf(uint8_t errorintack)
{
    ASSERT_ERR((((uint32_t)errorintack << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)errorintack << 8);
}

__INLINE uint8_t ble_intack_cryptintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ble_intack_cryptintack_clearf(uint8_t cryptintack)
{
    ASSERT_ERR((((uint32_t)cryptintack << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)cryptintack << 7);
}

__INLINE uint8_t ble_intack_skipevtintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void ble_intack_skipevtintack_clearf(uint8_t skipevtintack)
{
    ASSERT_ERR((((uint32_t)skipevtintack << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)skipevtintack << 6);
}

__INLINE uint8_t ble_intack_endevtintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void ble_intack_endevtintack_clearf(uint8_t endevtintack)
{
    ASSERT_ERR((((uint32_t)endevtintack << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)endevtintack << 5);
}

__INLINE uint8_t ble_intack_startevtintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ble_intack_startevtintack_clearf(uint8_t startevtintack)
{
    ASSERT_ERR((((uint32_t)startevtintack << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)startevtintack << 4);
}

__INLINE uint8_t ble_intack_slpintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_intack_slpintack_clearf(uint8_t slpintack)
{
    ASSERT_ERR((((uint32_t)slpintack << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)slpintack << 3);
}

__INLINE uint8_t ble_intack_rxintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_intack_rxintack_clearf(uint8_t rxintack)
{
    ASSERT_ERR((((uint32_t)rxintack << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)rxintack << 2);
}

__INLINE uint8_t ble_intack_txintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ble_intack_txintack_clearf(uint8_t txintack)
{
    ASSERT_ERR((((uint32_t)txintack << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)txintack << 1);
}

__INLINE uint8_t ble_intack_clknintack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_intack_clknintack_clearf(uint8_t clknintack)
{
    ASSERT_ERR((((uint32_t)clknintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_INTACK_ADDR, (uint32_t)clknintack << 0);
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
#define BLE_SLOTCLK_ADDR   0x4010001C
#define BLE_SLOTCLK_OFFSET 0x0000001C
#define BLE_SLOTCLK_INDEX  0x00000007
#define BLE_SLOTCLK_RESET  0x00000000

__INLINE uint32_t ble_slotclk_get(void)
{
    return REG_BLE_RD(BLE_SLOTCLK_ADDR);
}

__INLINE void ble_slotclk_set(uint32_t value)
{
    REG_BLE_WR(BLE_SLOTCLK_ADDR, value);
}

// field definitions
#define BLE_SAMP_BIT        ((uint32_t)0x80000000)
#define BLE_SAMP_POS        31
#define BLE_CLKN_UPD_BIT    ((uint32_t)0x40000000)
#define BLE_CLKN_UPD_POS    30
#define BLE_SCLK_MASK       ((uint32_t)0x0FFFFFFF)
#define BLE_SCLK_LSB        0
#define BLE_SCLK_WIDTH      ((uint32_t)0x0000001C)

#define BLE_SAMP_RST        0x0
#define BLE_CLKN_UPD_RST    0x0
#define BLE_SCLK_RST        0x0

__INLINE void ble_slotclk_pack(uint8_t samp, uint8_t clknupd, uint32_t sclk)
{
    ASSERT_ERR((((uint32_t)samp << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)clknupd << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)sclk << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BLE_WR(BLE_SLOTCLK_ADDR, ((uint32_t)samp << 31) | ((uint32_t)clknupd << 30) | ((uint32_t)sclk << 0));
}

__INLINE void ble_slotclk_unpack(uint8_t *samp, uint8_t *clknupd, uint32_t *sclk)
{
    uint32_t localVal = REG_BLE_RD(BLE_SLOTCLK_ADDR);

    *samp = (localVal & ((uint32_t)0x80000000)) >> 31;
    *clknupd = (localVal & ((uint32_t)0x40000000)) >> 30;
    *sclk = (localVal & ((uint32_t)0x0FFFFFFF)) >> 0;
}

__INLINE uint8_t ble_slotclk_samp_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_slotclk_samp_setf(uint8_t samp)
{
    ASSERT_ERR((((uint32_t)samp << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_SLOTCLK_ADDR, (REG_BLE_RD(BLE_SLOTCLK_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)samp << 31));
}

__INLINE uint8_t ble_slotclk_clkn_upd_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ble_slotclk_clkn_upd_setf(uint8_t clknupd)
{
    ASSERT_ERR((((uint32_t)clknupd << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BLE_WR(BLE_SLOTCLK_ADDR, (REG_BLE_RD(BLE_SLOTCLK_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)clknupd << 30));
}

__INLINE uint32_t ble_slotclk_sclk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x0FFFFFFF)) >> 0);
}

__INLINE void ble_slotclk_sclk_setf(uint32_t sclk)
{
    ASSERT_ERR((((uint32_t)sclk << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BLE_WR(BLE_SLOTCLK_ADDR, (REG_BLE_RD(BLE_SLOTCLK_ADDR) & ~((uint32_t)0x0FFFFFFF)) | ((uint32_t)sclk << 0));
}

/**
 * @brief FINETIMECNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00              FINECNT   0x0
 * </pre>
 */
#define BLE_FINETIMECNT_ADDR   0x40100020
#define BLE_FINETIMECNT_OFFSET 0x00000020
#define BLE_FINETIMECNT_INDEX  0x00000008
#define BLE_FINETIMECNT_RESET  0x00000000

__INLINE uint32_t ble_finetimecnt_get(void)
{
    return REG_BLE_RD(BLE_FINETIMECNT_ADDR);
}

// field definitions
#define BLE_FINECNT_MASK   ((uint32_t)0x000003FF)
#define BLE_FINECNT_LSB    0
#define BLE_FINECNT_WIDTH  ((uint32_t)0x0000000A)

#define BLE_FINECNT_RST    0x0

__INLINE uint16_t ble_finetimecnt_finecnt_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_FINETIMECNT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ET_CURRENTRXDESCPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16                ETPTR   0x0
 *  14:00     CURRENTRXDESCPTR   0x0
 * </pre>
 */
#define BLE_ET_CURRENTRXDESCPTR_ADDR   0x40100024
#define BLE_ET_CURRENTRXDESCPTR_OFFSET 0x00000024
#define BLE_ET_CURRENTRXDESCPTR_INDEX  0x00000009
#define BLE_ET_CURRENTRXDESCPTR_RESET  0x00000000

__INLINE uint32_t ble_et_currentrxdescptr_get(void)
{
    return REG_BLE_RD(BLE_ET_CURRENTRXDESCPTR_ADDR);
}

__INLINE void ble_et_currentrxdescptr_set(uint32_t value)
{
    REG_BLE_WR(BLE_ET_CURRENTRXDESCPTR_ADDR, value);
}

// field definitions
#define BLE_ETPTR_MASK              ((uint32_t)0xFFFF0000)
#define BLE_ETPTR_LSB               16
#define BLE_ETPTR_WIDTH             ((uint32_t)0x00000010)
#define BLE_CURRENTRXDESCPTR_MASK   ((uint32_t)0x00007FFF)
#define BLE_CURRENTRXDESCPTR_LSB    0
#define BLE_CURRENTRXDESCPTR_WIDTH  ((uint32_t)0x0000000F)

#define BLE_ETPTR_RST               0x0
#define BLE_CURRENTRXDESCPTR_RST    0x0

__INLINE void ble_et_currentrxdescptr_pack(uint16_t etptr, uint16_t currentrxdescptr)
{
    ASSERT_ERR((((uint32_t)etptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)currentrxdescptr << 0) & ~((uint32_t)0x00007FFF)) == 0);
    REG_BLE_WR(BLE_ET_CURRENTRXDESCPTR_ADDR, ((uint32_t)etptr << 16) | ((uint32_t)currentrxdescptr << 0));
}

__INLINE void ble_et_currentrxdescptr_unpack(uint16_t *etptr, uint16_t *currentrxdescptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_ET_CURRENTRXDESCPTR_ADDR);

    *etptr = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *currentrxdescptr = (localVal & ((uint32_t)0x00007FFF)) >> 0;
}

__INLINE uint16_t ble_et_currentrxdescptr_etptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ET_CURRENTRXDESCPTR_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_et_currentrxdescptr_etptr_setf(uint16_t etptr)
{
    ASSERT_ERR((((uint32_t)etptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_ET_CURRENTRXDESCPTR_ADDR, (REG_BLE_RD(BLE_ET_CURRENTRXDESCPTR_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)etptr << 16));
}

__INLINE uint16_t ble_et_currentrxdescptr_currentrxdescptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ET_CURRENTRXDESCPTR_ADDR);
    return ((localVal & ((uint32_t)0x00007FFF)) >> 0);
}

__INLINE void ble_et_currentrxdescptr_currentrxdescptr_setf(uint16_t currentrxdescptr)
{
    ASSERT_ERR((((uint32_t)currentrxdescptr << 0) & ~((uint32_t)0x00007FFF)) == 0);
    REG_BLE_WR(BLE_ET_CURRENTRXDESCPTR_ADDR, (REG_BLE_RD(BLE_ET_CURRENTRXDESCPTR_ADDR) & ~((uint32_t)0x00007FFF)) | ((uint32_t)currentrxdescptr << 0));
}

/**
 * @brief DEEPSLCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           EXTWKUPDSB   0
 *     15      DEEP_SLEEP_STAT   0
 *     04      SOFT_WAKEUP_REQ   0
 *     03   DEEP_SLEEP_CORR_EN   0
 *     02        DEEP_SLEEP_ON   0
 *     01       RADIO_SLEEP_EN   0
 *     00         OSC_SLEEP_EN   0
 * </pre>
 */
#define BLE_DEEPSLCNTL_ADDR   0x40100030
#define BLE_DEEPSLCNTL_OFFSET 0x00000030
#define BLE_DEEPSLCNTL_INDEX  0x0000000C
#define BLE_DEEPSLCNTL_RESET  0x00000000

__INLINE uint32_t ble_deepslcntl_get(void)
{
    return REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
}

__INLINE void ble_deepslcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, value);
}

// field definitions
#define BLE_EXTWKUPDSB_BIT            ((uint32_t)0x80000000)
#define BLE_EXTWKUPDSB_POS            31
#define BLE_DEEP_SLEEP_STAT_BIT       ((uint32_t)0x00008000)
#define BLE_DEEP_SLEEP_STAT_POS       15
#define BLE_SOFT_WAKEUP_REQ_BIT       ((uint32_t)0x00000010)
#define BLE_SOFT_WAKEUP_REQ_POS       4
#define BLE_DEEP_SLEEP_CORR_EN_BIT    ((uint32_t)0x00000008)
#define BLE_DEEP_SLEEP_CORR_EN_POS    3
#define BLE_DEEP_SLEEP_ON_BIT         ((uint32_t)0x00000004)
#define BLE_DEEP_SLEEP_ON_POS         2
#define BLE_RADIO_SLEEP_EN_BIT        ((uint32_t)0x00000002)
#define BLE_RADIO_SLEEP_EN_POS        1
#define BLE_OSC_SLEEP_EN_BIT          ((uint32_t)0x00000001)
#define BLE_OSC_SLEEP_EN_POS          0

#define BLE_EXTWKUPDSB_RST            0x0
#define BLE_DEEP_SLEEP_STAT_RST       0x0
#define BLE_SOFT_WAKEUP_REQ_RST       0x0
#define BLE_DEEP_SLEEP_CORR_EN_RST    0x0
#define BLE_DEEP_SLEEP_ON_RST         0x0
#define BLE_RADIO_SLEEP_EN_RST        0x0
#define BLE_OSC_SLEEP_EN_RST          0x0

__INLINE void ble_deepslcntl_pack(uint8_t extwkupdsb, uint8_t softwakeupreq, uint8_t deepsleepcorren, uint8_t deepsleepon, uint8_t radiosleepen, uint8_t oscsleepen)
{
    ASSERT_ERR((((uint32_t)extwkupdsb << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)softwakeupreq << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)deepsleepcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)deepsleepon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)radiosleepen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)oscsleepen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, ((uint32_t)extwkupdsb << 31) | ((uint32_t)softwakeupreq << 4) | ((uint32_t)deepsleepcorren << 3) | ((uint32_t)deepsleepon << 2) | ((uint32_t)radiosleepen << 1) | ((uint32_t)oscsleepen << 0));
}

__INLINE void ble_deepslcntl_unpack(uint8_t *extwkupdsb, uint8_t *deepsleepstat, uint8_t *softwakeupreq, uint8_t *deepsleepcorren, uint8_t *deepsleepon, uint8_t *radiosleepen, uint8_t *oscsleepen)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);

    *extwkupdsb = (localVal & ((uint32_t)0x80000000)) >> 31;
    *deepsleepstat = (localVal & ((uint32_t)0x00008000)) >> 15;
    *softwakeupreq = (localVal & ((uint32_t)0x00000010)) >> 4;
    *deepsleepcorren = (localVal & ((uint32_t)0x00000008)) >> 3;
    *deepsleepon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *radiosleepen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *oscsleepen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_deepslcntl_extwkupdsb_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_deepslcntl_extwkupdsb_setf(uint8_t extwkupdsb)
{
    ASSERT_ERR((((uint32_t)extwkupdsb << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, (REG_BLE_RD(BLE_DEEPSLCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)extwkupdsb << 31));
}

__INLINE uint8_t ble_deepslcntl_deep_sleep_stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t ble_deepslcntl_soft_wakeup_req_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ble_deepslcntl_soft_wakeup_req_setf(uint8_t softwakeupreq)
{
    ASSERT_ERR((((uint32_t)softwakeupreq << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, (REG_BLE_RD(BLE_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)softwakeupreq << 4));
}

__INLINE uint8_t ble_deepslcntl_deep_sleep_corr_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_deepslcntl_deep_sleep_corr_en_setf(uint8_t deepsleepcorren)
{
    ASSERT_ERR((((uint32_t)deepsleepcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, (REG_BLE_RD(BLE_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)deepsleepcorren << 3));
}

__INLINE uint8_t ble_deepslcntl_deep_sleep_on_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_deepslcntl_deep_sleep_on_setf(uint8_t deepsleepon)
{
    ASSERT_ERR((((uint32_t)deepsleepon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, (REG_BLE_RD(BLE_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)deepsleepon << 2));
}

__INLINE uint8_t ble_deepslcntl_radio_sleep_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ble_deepslcntl_radio_sleep_en_setf(uint8_t radiosleepen)
{
    ASSERT_ERR((((uint32_t)radiosleepen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, (REG_BLE_RD(BLE_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)radiosleepen << 1));
}

__INLINE uint8_t ble_deepslcntl_osc_sleep_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_deepslcntl_osc_sleep_en_setf(uint8_t oscsleepen)
{
    ASSERT_ERR((((uint32_t)oscsleepen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_DEEPSLCNTL_ADDR, (REG_BLE_RD(BLE_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)oscsleepen << 0));
}

/**
 * @brief DEEPSLWKUP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           DEEPSLTIME   0x0
 * </pre>
 */
#define BLE_DEEPSLWKUP_ADDR   0x40100034
#define BLE_DEEPSLWKUP_OFFSET 0x00000034
#define BLE_DEEPSLWKUP_INDEX  0x0000000D
#define BLE_DEEPSLWKUP_RESET  0x00000000

__INLINE uint32_t ble_deepslwkup_get(void)
{
    return REG_BLE_RD(BLE_DEEPSLWKUP_ADDR);
}

__INLINE void ble_deepslwkup_set(uint32_t value)
{
    REG_BLE_WR(BLE_DEEPSLWKUP_ADDR, value);
}

// field definitions
#define BLE_DEEPSLTIME_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_DEEPSLTIME_LSB    0
#define BLE_DEEPSLTIME_WIDTH  ((uint32_t)0x00000020)

#define BLE_DEEPSLTIME_RST    0x0

__INLINE uint32_t ble_deepslwkup_deepsltime_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLWKUP_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_deepslwkup_deepsltime_setf(uint32_t deepsltime)
{
    ASSERT_ERR((((uint32_t)deepsltime << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_DEEPSLWKUP_ADDR, (uint32_t)deepsltime << 0);
}

/**
 * @brief DEEPSLSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            DEEPSLDUR   0x0
 * </pre>
 */
#define BLE_DEEPSLSTAT_ADDR   0x40100038
#define BLE_DEEPSLSTAT_OFFSET 0x00000038
#define BLE_DEEPSLSTAT_INDEX  0x0000000E
#define BLE_DEEPSLSTAT_RESET  0x00000000

__INLINE uint32_t ble_deepslstat_get(void)
{
    return REG_BLE_RD(BLE_DEEPSLSTAT_ADDR);
}

// field definitions
#define BLE_DEEPSLDUR_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_DEEPSLDUR_LSB    0
#define BLE_DEEPSLDUR_WIDTH  ((uint32_t)0x00000020)

#define BLE_DEEPSLDUR_RST    0x0

__INLINE uint32_t ble_deepslstat_deepsldur_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEEPSLSTAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ENBPRESET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:21                TWEXT   0x0
 *  20:10                TWOSC   0x0
 *  09:00                 TWRM   0x0
 * </pre>
 */
#define BLE_ENBPRESET_ADDR   0x4010003C
#define BLE_ENBPRESET_OFFSET 0x0000003C
#define BLE_ENBPRESET_INDEX  0x0000000F
#define BLE_ENBPRESET_RESET  0x00000000

__INLINE uint32_t ble_enbpreset_get(void)
{
    return REG_BLE_RD(BLE_ENBPRESET_ADDR);
}

__INLINE void ble_enbpreset_set(uint32_t value)
{
    REG_BLE_WR(BLE_ENBPRESET_ADDR, value);
}

// field definitions
#define BLE_TWEXT_MASK   ((uint32_t)0xFFE00000)
#define BLE_TWEXT_LSB    21
#define BLE_TWEXT_WIDTH  ((uint32_t)0x0000000B)
#define BLE_TWOSC_MASK   ((uint32_t)0x001FFC00)
#define BLE_TWOSC_LSB    10
#define BLE_TWOSC_WIDTH  ((uint32_t)0x0000000B)
#define BLE_TWRM_MASK    ((uint32_t)0x000003FF)
#define BLE_TWRM_LSB     0
#define BLE_TWRM_WIDTH   ((uint32_t)0x0000000A)

#define BLE_TWEXT_RST    0x0
#define BLE_TWOSC_RST    0x0
#define BLE_TWRM_RST     0x0

__INLINE void ble_enbpreset_pack(uint16_t twext, uint16_t twosc, uint16_t twrm)
{
    ASSERT_ERR((((uint32_t)twext << 21) & ~((uint32_t)0xFFE00000)) == 0);
    ASSERT_ERR((((uint32_t)twosc << 10) & ~((uint32_t)0x001FFC00)) == 0);
    ASSERT_ERR((((uint32_t)twrm << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BLE_WR(BLE_ENBPRESET_ADDR, ((uint32_t)twext << 21) | ((uint32_t)twosc << 10) | ((uint32_t)twrm << 0));
}

__INLINE void ble_enbpreset_unpack(uint16_t *twext, uint16_t *twosc, uint16_t *twrm)
{
    uint32_t localVal = REG_BLE_RD(BLE_ENBPRESET_ADDR);

    *twext = (localVal & ((uint32_t)0xFFE00000)) >> 21;
    *twosc = (localVal & ((uint32_t)0x001FFC00)) >> 10;
    *twrm = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t ble_enbpreset_twext_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0xFFE00000)) >> 21);
}

__INLINE void ble_enbpreset_twext_setf(uint16_t twext)
{
    ASSERT_ERR((((uint32_t)twext << 21) & ~((uint32_t)0xFFE00000)) == 0);
    REG_BLE_WR(BLE_ENBPRESET_ADDR, (REG_BLE_RD(BLE_ENBPRESET_ADDR) & ~((uint32_t)0xFFE00000)) | ((uint32_t)twext << 21));
}

__INLINE uint16_t ble_enbpreset_twosc_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0x001FFC00)) >> 10);
}

__INLINE void ble_enbpreset_twosc_setf(uint16_t twosc)
{
    ASSERT_ERR((((uint32_t)twosc << 10) & ~((uint32_t)0x001FFC00)) == 0);
    REG_BLE_WR(BLE_ENBPRESET_ADDR, (REG_BLE_RD(BLE_ENBPRESET_ADDR) & ~((uint32_t)0x001FFC00)) | ((uint32_t)twosc << 10));
}

__INLINE uint16_t ble_enbpreset_twrm_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void ble_enbpreset_twrm_setf(uint16_t twrm)
{
    ASSERT_ERR((((uint32_t)twrm << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BLE_WR(BLE_ENBPRESET_ADDR, (REG_BLE_RD(BLE_ENBPRESET_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)twrm << 0));
}

/**
 * @brief FINECNTCORR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00          FINECNTCORR   0x0
 * </pre>
 */
#define BLE_FINECNTCORR_ADDR   0x40100040
#define BLE_FINECNTCORR_OFFSET 0x00000040
#define BLE_FINECNTCORR_INDEX  0x00000010
#define BLE_FINECNTCORR_RESET  0x00000000

__INLINE uint32_t ble_finecntcorr_get(void)
{
    return REG_BLE_RD(BLE_FINECNTCORR_ADDR);
}

__INLINE void ble_finecntcorr_set(uint32_t value)
{
    REG_BLE_WR(BLE_FINECNTCORR_ADDR, value);
}

// field definitions
#define BLE_FINECNTCORR_MASK   ((uint32_t)0x000003FF)
#define BLE_FINECNTCORR_LSB    0
#define BLE_FINECNTCORR_WIDTH  ((uint32_t)0x0000000A)

#define BLE_FINECNTCORR_RST    0x0

__INLINE uint16_t ble_finecntcorr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_FINECNTCORR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_finecntcorr_setf(uint16_t finecntcorr)
{
    ASSERT_ERR((((uint32_t)finecntcorr << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BLE_WR(BLE_FINECNTCORR_ADDR, (uint32_t)finecntcorr << 0);
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
#define BLE_CLKNCNTCORR_ADDR   0x40100044
#define BLE_CLKNCNTCORR_OFFSET 0x00000044
#define BLE_CLKNCNTCORR_INDEX  0x00000011
#define BLE_CLKNCNTCORR_RESET  0x00000000

__INLINE uint32_t ble_clkncntcorr_get(void)
{
    return REG_BLE_RD(BLE_CLKNCNTCORR_ADDR);
}

__INLINE void ble_clkncntcorr_set(uint32_t value)
{
    REG_BLE_WR(BLE_CLKNCNTCORR_ADDR, value);
}

// field definitions
#define BLE_ABS_DELTA_BIT      ((uint32_t)0x80000000)
#define BLE_ABS_DELTA_POS      31
#define BLE_CLKNCNTCORR_MASK   ((uint32_t)0x0FFFFFFF)
#define BLE_CLKNCNTCORR_LSB    0
#define BLE_CLKNCNTCORR_WIDTH  ((uint32_t)0x0000001C)

#define BLE_ABS_DELTA_RST      0x0
#define BLE_CLKNCNTCORR_RST    0x0

__INLINE void ble_clkncntcorr_pack(uint8_t absdelta, uint32_t clkncntcorr)
{
    ASSERT_ERR((((uint32_t)absdelta << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)clkncntcorr << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BLE_WR(BLE_CLKNCNTCORR_ADDR, ((uint32_t)absdelta << 31) | ((uint32_t)clkncntcorr << 0));
}

__INLINE void ble_clkncntcorr_unpack(uint8_t *absdelta, uint32_t *clkncntcorr)
{
    uint32_t localVal = REG_BLE_RD(BLE_CLKNCNTCORR_ADDR);

    *absdelta = (localVal & ((uint32_t)0x80000000)) >> 31;
    *clkncntcorr = (localVal & ((uint32_t)0x0FFFFFFF)) >> 0;
}

__INLINE uint8_t ble_clkncntcorr_abs_delta_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_CLKNCNTCORR_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_clkncntcorr_abs_delta_setf(uint8_t absdelta)
{
    ASSERT_ERR((((uint32_t)absdelta << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_CLKNCNTCORR_ADDR, (REG_BLE_RD(BLE_CLKNCNTCORR_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)absdelta << 31));
}

__INLINE uint32_t ble_clkncntcorr_clkncntcorr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_CLKNCNTCORR_ADDR);
    return ((localVal & ((uint32_t)0x0FFFFFFF)) >> 0);
}

__INLINE void ble_clkncntcorr_clkncntcorr_setf(uint32_t clkncntcorr)
{
    ASSERT_ERR((((uint32_t)clkncntcorr << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BLE_WR(BLE_CLKNCNTCORR_ADDR, (REG_BLE_RD(BLE_CLKNCNTCORR_ADDR) & ~((uint32_t)0x0FFFFFFF)) | ((uint32_t)clkncntcorr << 0));
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
#define BLE_DIAGCNTL_ADDR   0x40100050
#define BLE_DIAGCNTL_OFFSET 0x00000050
#define BLE_DIAGCNTL_INDEX  0x00000014
#define BLE_DIAGCNTL_RESET  0x00000000

__INLINE uint32_t ble_diagcntl_get(void)
{
    return REG_BLE_RD(BLE_DIAGCNTL_ADDR);
}

__INLINE void ble_diagcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, value);
}

// field definitions
#define BLE_DIAG3_EN_BIT    ((uint32_t)0x80000000)
#define BLE_DIAG3_EN_POS    31
#define BLE_DIAG3_MASK      ((uint32_t)0x7F000000)
#define BLE_DIAG3_LSB       24
#define BLE_DIAG3_WIDTH     ((uint32_t)0x00000007)
#define BLE_DIAG2_EN_BIT    ((uint32_t)0x00800000)
#define BLE_DIAG2_EN_POS    23
#define BLE_DIAG2_MASK      ((uint32_t)0x007F0000)
#define BLE_DIAG2_LSB       16
#define BLE_DIAG2_WIDTH     ((uint32_t)0x00000007)
#define BLE_DIAG1_EN_BIT    ((uint32_t)0x00008000)
#define BLE_DIAG1_EN_POS    15
#define BLE_DIAG1_MASK      ((uint32_t)0x00007F00)
#define BLE_DIAG1_LSB       8
#define BLE_DIAG1_WIDTH     ((uint32_t)0x00000007)
#define BLE_DIAG0_EN_BIT    ((uint32_t)0x00000080)
#define BLE_DIAG0_EN_POS    7
#define BLE_DIAG0_MASK      ((uint32_t)0x0000007F)
#define BLE_DIAG0_LSB       0
#define BLE_DIAG0_WIDTH     ((uint32_t)0x00000007)

#define BLE_DIAG3_EN_RST    0x0
#define BLE_DIAG3_RST       0x0
#define BLE_DIAG2_EN_RST    0x0
#define BLE_DIAG2_RST       0x0
#define BLE_DIAG1_EN_RST    0x0
#define BLE_DIAG1_RST       0x0
#define BLE_DIAG0_EN_RST    0x0
#define BLE_DIAG0_RST       0x0

__INLINE void ble_diagcntl_pack(uint8_t diag3en, uint8_t diag3, uint8_t diag2en, uint8_t diag2, uint8_t diag1en, uint8_t diag1, uint8_t diag0en, uint8_t diag0)
{
    ASSERT_ERR((((uint32_t)diag3en << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)diag3 << 24) & ~((uint32_t)0x7F000000)) == 0);
    ASSERT_ERR((((uint32_t)diag2en << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)diag2 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)diag1en << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)diag1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)diag0en << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)diag0 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, ((uint32_t)diag3en << 31) | ((uint32_t)diag3 << 24) | ((uint32_t)diag2en << 23) | ((uint32_t)diag2 << 16) | ((uint32_t)diag1en << 15) | ((uint32_t)diag1 << 8) | ((uint32_t)diag0en << 7) | ((uint32_t)diag0 << 0));
}

__INLINE void ble_diagcntl_unpack(uint8_t *diag3en, uint8_t *diag3, uint8_t *diag2en, uint8_t *diag2, uint8_t *diag1en, uint8_t *diag1, uint8_t *diag0en, uint8_t *diag0)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);

    *diag3en = (localVal & ((uint32_t)0x80000000)) >> 31;
    *diag3 = (localVal & ((uint32_t)0x7F000000)) >> 24;
    *diag2en = (localVal & ((uint32_t)0x00800000)) >> 23;
    *diag2 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *diag1en = (localVal & ((uint32_t)0x00008000)) >> 15;
    *diag1 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *diag0en = (localVal & ((uint32_t)0x00000080)) >> 7;
    *diag0 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t ble_diagcntl_diag3_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_diagcntl_diag3_en_setf(uint8_t diag3en)
{
    ASSERT_ERR((((uint32_t)diag3en << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)diag3en << 31));
}

__INLINE uint8_t ble_diagcntl_diag3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x7F000000)) >> 24);
}

__INLINE void ble_diagcntl_diag3_setf(uint8_t diag3)
{
    ASSERT_ERR((((uint32_t)diag3 << 24) & ~((uint32_t)0x7F000000)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x7F000000)) | ((uint32_t)diag3 << 24));
}

__INLINE uint8_t ble_diagcntl_diag2_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void ble_diagcntl_diag2_en_setf(uint8_t diag2en)
{
    ASSERT_ERR((((uint32_t)diag2en << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)diag2en << 23));
}

__INLINE uint8_t ble_diagcntl_diag2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void ble_diagcntl_diag2_setf(uint8_t diag2)
{
    ASSERT_ERR((((uint32_t)diag2 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)diag2 << 16));
}

__INLINE uint8_t ble_diagcntl_diag1_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ble_diagcntl_diag1_en_setf(uint8_t diag1en)
{
    ASSERT_ERR((((uint32_t)diag1en << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)diag1en << 15));
}

__INLINE uint8_t ble_diagcntl_diag1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_diagcntl_diag1_setf(uint8_t diag1)
{
    ASSERT_ERR((((uint32_t)diag1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)diag1 << 8));
}

__INLINE uint8_t ble_diagcntl_diag0_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ble_diagcntl_diag0_en_setf(uint8_t diag0en)
{
    ASSERT_ERR((((uint32_t)diag0en << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)diag0en << 7));
}

__INLINE uint8_t ble_diagcntl_diag0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void ble_diagcntl_diag0_setf(uint8_t diag0)
{
    ASSERT_ERR((((uint32_t)diag0 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_DIAGCNTL_ADDR, (REG_BLE_RD(BLE_DIAGCNTL_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)diag0 << 0));
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
#define BLE_DIAGSTAT_ADDR   0x40100054
#define BLE_DIAGSTAT_OFFSET 0x00000054
#define BLE_DIAGSTAT_INDEX  0x00000015
#define BLE_DIAGSTAT_RESET  0x00000000

__INLINE uint32_t ble_diagstat_get(void)
{
    return REG_BLE_RD(BLE_DIAGSTAT_ADDR);
}

// field definitions
#define BLE_DIAG3STAT_MASK   ((uint32_t)0xFF000000)
#define BLE_DIAG3STAT_LSB    24
#define BLE_DIAG3STAT_WIDTH  ((uint32_t)0x00000008)
#define BLE_DIAG2STAT_MASK   ((uint32_t)0x00FF0000)
#define BLE_DIAG2STAT_LSB    16
#define BLE_DIAG2STAT_WIDTH  ((uint32_t)0x00000008)
#define BLE_DIAG1STAT_MASK   ((uint32_t)0x0000FF00)
#define BLE_DIAG1STAT_LSB    8
#define BLE_DIAG1STAT_WIDTH  ((uint32_t)0x00000008)
#define BLE_DIAG0STAT_MASK   ((uint32_t)0x000000FF)
#define BLE_DIAG0STAT_LSB    0
#define BLE_DIAG0STAT_WIDTH  ((uint32_t)0x00000008)

#define BLE_DIAG3STAT_RST    0x0
#define BLE_DIAG2STAT_RST    0x0
#define BLE_DIAG1STAT_RST    0x0
#define BLE_DIAG0STAT_RST    0x0

__INLINE void ble_diagstat_unpack(uint8_t *diag3stat, uint8_t *diag2stat, uint8_t *diag1stat, uint8_t *diag0stat)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGSTAT_ADDR);

    *diag3stat = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *diag2stat = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *diag1stat = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *diag0stat = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_diagstat_diag3stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t ble_diagstat_diag2stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t ble_diagstat_diag1stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t ble_diagstat_diag0stat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DIAGSTAT_ADDR);
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
#define BLE_DEBUGADDMAX_ADDR   0x40100058
#define BLE_DEBUGADDMAX_OFFSET 0x00000058
#define BLE_DEBUGADDMAX_INDEX  0x00000016
#define BLE_DEBUGADDMAX_RESET  0x00000000

__INLINE uint32_t ble_debugaddmax_get(void)
{
    return REG_BLE_RD(BLE_DEBUGADDMAX_ADDR);
}

__INLINE void ble_debugaddmax_set(uint32_t value)
{
    REG_BLE_WR(BLE_DEBUGADDMAX_ADDR, value);
}

// field definitions
#define BLE_REG_ADDMAX_MASK   ((uint32_t)0xFFFF0000)
#define BLE_REG_ADDMAX_LSB    16
#define BLE_REG_ADDMAX_WIDTH  ((uint32_t)0x00000010)
#define BLE_EM_ADDMAX_MASK    ((uint32_t)0x0000FFFF)
#define BLE_EM_ADDMAX_LSB     0
#define BLE_EM_ADDMAX_WIDTH   ((uint32_t)0x00000010)

#define BLE_REG_ADDMAX_RST    0x0
#define BLE_EM_ADDMAX_RST     0x0

__INLINE void ble_debugaddmax_pack(uint16_t regaddmax, uint16_t emaddmax)
{
    ASSERT_ERR((((uint32_t)regaddmax << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)emaddmax << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_DEBUGADDMAX_ADDR, ((uint32_t)regaddmax << 16) | ((uint32_t)emaddmax << 0));
}

__INLINE void ble_debugaddmax_unpack(uint16_t *regaddmax, uint16_t *emaddmax)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEBUGADDMAX_ADDR);

    *regaddmax = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *emaddmax = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_debugaddmax_reg_addmax_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEBUGADDMAX_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_debugaddmax_reg_addmax_setf(uint16_t regaddmax)
{
    ASSERT_ERR((((uint32_t)regaddmax << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_DEBUGADDMAX_ADDR, (REG_BLE_RD(BLE_DEBUGADDMAX_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)regaddmax << 16));
}

__INLINE uint16_t ble_debugaddmax_em_addmax_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEBUGADDMAX_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_debugaddmax_em_addmax_setf(uint16_t emaddmax)
{
    ASSERT_ERR((((uint32_t)emaddmax << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_DEBUGADDMAX_ADDR, (REG_BLE_RD(BLE_DEBUGADDMAX_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)emaddmax << 0));
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
#define BLE_DEBUGADDMIN_ADDR   0x4010005C
#define BLE_DEBUGADDMIN_OFFSET 0x0000005C
#define BLE_DEBUGADDMIN_INDEX  0x00000017
#define BLE_DEBUGADDMIN_RESET  0x00000000

__INLINE uint32_t ble_debugaddmin_get(void)
{
    return REG_BLE_RD(BLE_DEBUGADDMIN_ADDR);
}

__INLINE void ble_debugaddmin_set(uint32_t value)
{
    REG_BLE_WR(BLE_DEBUGADDMIN_ADDR, value);
}

// field definitions
#define BLE_REG_ADDMIN_MASK   ((uint32_t)0xFFFF0000)
#define BLE_REG_ADDMIN_LSB    16
#define BLE_REG_ADDMIN_WIDTH  ((uint32_t)0x00000010)
#define BLE_EM_ADDMIN_MASK    ((uint32_t)0x0000FFFF)
#define BLE_EM_ADDMIN_LSB     0
#define BLE_EM_ADDMIN_WIDTH   ((uint32_t)0x00000010)

#define BLE_REG_ADDMIN_RST    0x0
#define BLE_EM_ADDMIN_RST     0x0

__INLINE void ble_debugaddmin_pack(uint16_t regaddmin, uint16_t emaddmin)
{
    ASSERT_ERR((((uint32_t)regaddmin << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)emaddmin << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_DEBUGADDMIN_ADDR, ((uint32_t)regaddmin << 16) | ((uint32_t)emaddmin << 0));
}

__INLINE void ble_debugaddmin_unpack(uint16_t *regaddmin, uint16_t *emaddmin)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEBUGADDMIN_ADDR);

    *regaddmin = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *emaddmin = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_debugaddmin_reg_addmin_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEBUGADDMIN_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_debugaddmin_reg_addmin_setf(uint16_t regaddmin)
{
    ASSERT_ERR((((uint32_t)regaddmin << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_DEBUGADDMIN_ADDR, (REG_BLE_RD(BLE_DEBUGADDMIN_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)regaddmin << 16));
}

__INLINE uint16_t ble_debugaddmin_em_addmin_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_DEBUGADDMIN_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_debugaddmin_em_addmin_setf(uint16_t emaddmin)
{
    ASSERT_ERR((((uint32_t)emaddmin << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_DEBUGADDMIN_ADDR, (REG_BLE_RD(BLE_DEBUGADDMIN_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)emaddmin << 0));
}

/**
 * @brief ERRORTYPESTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     20            PHY_ERROR   0
 *     19   TXAEHEADER_PTR_ERROR   0
 *     18       TMAFS_UNDERRUN   0
 *     17         RAL_UNDERRUN   0
 *     16            RAL_ERROR   0
 *     15     RXDATA_PTR_ERROR   0
 *     14     TXDATA_PTR_ERROR   0
 *     13   RXDESC_EMPTY_ERROR   0
 *     12   TXDESC_EMPTY_ERROR   0
 *     11       CSFORMAT_ERROR   0
 *     10        LLCHMAP_ERROR   0
 *     09         ADV_UNDERRUN   0
 *     08         IFS_UNDERRUN   0
 *     07           LIST_ERROR   0
 *     06   EVT_CNTL_APFM_ERROR   0
 *     05   EVT_SCHDL_APFM_ERROR   0
 *     04   EVT_SCHDL_ENTRY_ERROR   0
 *     03    RADIO_EMACC_ERROR   0
 *     02   PKTCNTL_EMACC_ERROR   0
 *     01        RXCRYPT_ERROR   0
 *     00        TXCRYPT_ERROR   0
 * </pre>
 */
#define BLE_ERRORTYPESTAT_ADDR   0x40100060
#define BLE_ERRORTYPESTAT_OFFSET 0x00000060
#define BLE_ERRORTYPESTAT_INDEX  0x00000018
#define BLE_ERRORTYPESTAT_RESET  0x00000000

__INLINE uint32_t ble_errortypestat_get(void)
{
    return REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
}

// field definitions
#define BLE_PHY_ERROR_BIT                ((uint32_t)0x00100000)
#define BLE_PHY_ERROR_POS                20
#define BLE_TXAEHEADER_PTR_ERROR_BIT     ((uint32_t)0x00080000)
#define BLE_TXAEHEADER_PTR_ERROR_POS     19
#define BLE_TMAFS_UNDERRUN_BIT           ((uint32_t)0x00040000)
#define BLE_TMAFS_UNDERRUN_POS           18
#define BLE_RAL_UNDERRUN_BIT             ((uint32_t)0x00020000)
#define BLE_RAL_UNDERRUN_POS             17
#define BLE_RAL_ERROR_BIT                ((uint32_t)0x00010000)
#define BLE_RAL_ERROR_POS                16
#define BLE_RXDATA_PTR_ERROR_BIT         ((uint32_t)0x00008000)
#define BLE_RXDATA_PTR_ERROR_POS         15
#define BLE_TXDATA_PTR_ERROR_BIT         ((uint32_t)0x00004000)
#define BLE_TXDATA_PTR_ERROR_POS         14
#define BLE_RXDESC_EMPTY_ERROR_BIT       ((uint32_t)0x00002000)
#define BLE_RXDESC_EMPTY_ERROR_POS       13
#define BLE_TXDESC_EMPTY_ERROR_BIT       ((uint32_t)0x00001000)
#define BLE_TXDESC_EMPTY_ERROR_POS       12
#define BLE_CSFORMAT_ERROR_BIT           ((uint32_t)0x00000800)
#define BLE_CSFORMAT_ERROR_POS           11
#define BLE_LLCHMAP_ERROR_BIT            ((uint32_t)0x00000400)
#define BLE_LLCHMAP_ERROR_POS            10
#define BLE_ADV_UNDERRUN_BIT             ((uint32_t)0x00000200)
#define BLE_ADV_UNDERRUN_POS             9
#define BLE_IFS_UNDERRUN_BIT             ((uint32_t)0x00000100)
#define BLE_IFS_UNDERRUN_POS             8
#define BLE_LIST_ERROR_BIT               ((uint32_t)0x00000080)
#define BLE_LIST_ERROR_POS               7
#define BLE_EVT_CNTL_APFM_ERROR_BIT      ((uint32_t)0x00000040)
#define BLE_EVT_CNTL_APFM_ERROR_POS      6
#define BLE_EVT_SCHDL_APFM_ERROR_BIT     ((uint32_t)0x00000020)
#define BLE_EVT_SCHDL_APFM_ERROR_POS     5
#define BLE_EVT_SCHDL_ENTRY_ERROR_BIT    ((uint32_t)0x00000010)
#define BLE_EVT_SCHDL_ENTRY_ERROR_POS    4
#define BLE_RADIO_EMACC_ERROR_BIT        ((uint32_t)0x00000008)
#define BLE_RADIO_EMACC_ERROR_POS        3
#define BLE_PKTCNTL_EMACC_ERROR_BIT      ((uint32_t)0x00000004)
#define BLE_PKTCNTL_EMACC_ERROR_POS      2
#define BLE_RXCRYPT_ERROR_BIT            ((uint32_t)0x00000002)
#define BLE_RXCRYPT_ERROR_POS            1
#define BLE_TXCRYPT_ERROR_BIT            ((uint32_t)0x00000001)
#define BLE_TXCRYPT_ERROR_POS            0

#define BLE_PHY_ERROR_RST                0x0
#define BLE_TXAEHEADER_PTR_ERROR_RST     0x0
#define BLE_TMAFS_UNDERRUN_RST           0x0
#define BLE_RAL_UNDERRUN_RST             0x0
#define BLE_RAL_ERROR_RST                0x0
#define BLE_RXDATA_PTR_ERROR_RST         0x0
#define BLE_TXDATA_PTR_ERROR_RST         0x0
#define BLE_RXDESC_EMPTY_ERROR_RST       0x0
#define BLE_TXDESC_EMPTY_ERROR_RST       0x0
#define BLE_CSFORMAT_ERROR_RST           0x0
#define BLE_LLCHMAP_ERROR_RST            0x0
#define BLE_ADV_UNDERRUN_RST             0x0
#define BLE_IFS_UNDERRUN_RST             0x0
#define BLE_LIST_ERROR_RST               0x0
#define BLE_EVT_CNTL_APFM_ERROR_RST      0x0
#define BLE_EVT_SCHDL_APFM_ERROR_RST     0x0
#define BLE_EVT_SCHDL_ENTRY_ERROR_RST    0x0
#define BLE_RADIO_EMACC_ERROR_RST        0x0
#define BLE_PKTCNTL_EMACC_ERROR_RST      0x0
#define BLE_RXCRYPT_ERROR_RST            0x0
#define BLE_TXCRYPT_ERROR_RST            0x0

__INLINE void ble_errortypestat_unpack(uint8_t *phyerror, uint8_t *txaeheaderptrerror, uint8_t *tmafsunderrun, uint8_t *ralunderrun, uint8_t *ralerror, uint8_t *rxdataptrerror, uint8_t *txdataptrerror, uint8_t *rxdescemptyerror, uint8_t *txdescemptyerror, uint8_t *csformaterror, uint8_t *llchmaperror, uint8_t *advunderrun, uint8_t *ifsunderrun, uint8_t *listerror, uint8_t *evtcntlapfmerror, uint8_t *evtschdlapfmerror, uint8_t *evtschdlentryerror, uint8_t *radioemaccerror, uint8_t *pktcntlemaccerror, uint8_t *rxcrypterror, uint8_t *txcrypterror)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);

    *phyerror = (localVal & ((uint32_t)0x00100000)) >> 20;
    *txaeheaderptrerror = (localVal & ((uint32_t)0x00080000)) >> 19;
    *tmafsunderrun = (localVal & ((uint32_t)0x00040000)) >> 18;
    *ralunderrun = (localVal & ((uint32_t)0x00020000)) >> 17;
    *ralerror = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rxdataptrerror = (localVal & ((uint32_t)0x00008000)) >> 15;
    *txdataptrerror = (localVal & ((uint32_t)0x00004000)) >> 14;
    *rxdescemptyerror = (localVal & ((uint32_t)0x00002000)) >> 13;
    *txdescemptyerror = (localVal & ((uint32_t)0x00001000)) >> 12;
    *csformaterror = (localVal & ((uint32_t)0x00000800)) >> 11;
    *llchmaperror = (localVal & ((uint32_t)0x00000400)) >> 10;
    *advunderrun = (localVal & ((uint32_t)0x00000200)) >> 9;
    *ifsunderrun = (localVal & ((uint32_t)0x00000100)) >> 8;
    *listerror = (localVal & ((uint32_t)0x00000080)) >> 7;
    *evtcntlapfmerror = (localVal & ((uint32_t)0x00000040)) >> 6;
    *evtschdlapfmerror = (localVal & ((uint32_t)0x00000020)) >> 5;
    *evtschdlentryerror = (localVal & ((uint32_t)0x00000010)) >> 4;
    *radioemaccerror = (localVal & ((uint32_t)0x00000008)) >> 3;
    *pktcntlemaccerror = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxcrypterror = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txcrypterror = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_errortypestat_phy_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE uint8_t ble_errortypestat_txaeheader_ptr_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE uint8_t ble_errortypestat_tmafs_underrun_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE uint8_t ble_errortypestat_ral_underrun_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE uint8_t ble_errortypestat_ral_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint8_t ble_errortypestat_rxdata_ptr_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t ble_errortypestat_txdata_ptr_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t ble_errortypestat_rxdesc_empty_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t ble_errortypestat_txdesc_empty_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t ble_errortypestat_csformat_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t ble_errortypestat_llchmap_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t ble_errortypestat_adv_underrun_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t ble_errortypestat_ifs_underrun_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t ble_errortypestat_list_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t ble_errortypestat_evt_cntl_apfm_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t ble_errortypestat_evt_schdl_apfm_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t ble_errortypestat_evt_schdl_entry_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t ble_errortypestat_radio_emacc_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t ble_errortypestat_pktcntl_emacc_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ble_errortypestat_rxcrypt_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ble_errortypestat_txcrypt_error_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ERRORTYPESTAT_ADDR);
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
#define BLE_SWPROFILING_ADDR   0x40100064
#define BLE_SWPROFILING_OFFSET 0x00000064
#define BLE_SWPROFILING_INDEX  0x00000019
#define BLE_SWPROFILING_RESET  0x00000000

__INLINE uint32_t ble_swprofiling_get(void)
{
    return REG_BLE_RD(BLE_SWPROFILING_ADDR);
}

__INLINE void ble_swprofiling_set(uint32_t value)
{
    REG_BLE_WR(BLE_SWPROFILING_ADDR, value);
}

// field definitions
#define BLE_SWPROF31_BIT    ((uint32_t)0x80000000)
#define BLE_SWPROF31_POS    31
#define BLE_SWPROF30_BIT    ((uint32_t)0x40000000)
#define BLE_SWPROF30_POS    30
#define BLE_SWPROF29_BIT    ((uint32_t)0x20000000)
#define BLE_SWPROF29_POS    29
#define BLE_SWPROF28_BIT    ((uint32_t)0x10000000)
#define BLE_SWPROF28_POS    28
#define BLE_SWPROF27_BIT    ((uint32_t)0x08000000)
#define BLE_SWPROF27_POS    27
#define BLE_SWPROF26_BIT    ((uint32_t)0x04000000)
#define BLE_SWPROF26_POS    26
#define BLE_SWPROF25_BIT    ((uint32_t)0x02000000)
#define BLE_SWPROF25_POS    25
#define BLE_SWPROF24_BIT    ((uint32_t)0x01000000)
#define BLE_SWPROF24_POS    24
#define BLE_SWPROF23_BIT    ((uint32_t)0x00800000)
#define BLE_SWPROF23_POS    23
#define BLE_SWPROF22_BIT    ((uint32_t)0x00400000)
#define BLE_SWPROF22_POS    22
#define BLE_SWPROF21_BIT    ((uint32_t)0x00200000)
#define BLE_SWPROF21_POS    21
#define BLE_SWPROF20_BIT    ((uint32_t)0x00100000)
#define BLE_SWPROF20_POS    20
#define BLE_SWPROF19_BIT    ((uint32_t)0x00080000)
#define BLE_SWPROF19_POS    19
#define BLE_SWPROF18_BIT    ((uint32_t)0x00040000)
#define BLE_SWPROF18_POS    18
#define BLE_SWPROF17_BIT    ((uint32_t)0x00020000)
#define BLE_SWPROF17_POS    17
#define BLE_SWPROF16_BIT    ((uint32_t)0x00010000)
#define BLE_SWPROF16_POS    16
#define BLE_SWPROF15_BIT    ((uint32_t)0x00008000)
#define BLE_SWPROF15_POS    15
#define BLE_SWPROF14_BIT    ((uint32_t)0x00004000)
#define BLE_SWPROF14_POS    14
#define BLE_SWPROF13_BIT    ((uint32_t)0x00002000)
#define BLE_SWPROF13_POS    13
#define BLE_SWPROF12_BIT    ((uint32_t)0x00001000)
#define BLE_SWPROF12_POS    12
#define BLE_SWPROF11_BIT    ((uint32_t)0x00000800)
#define BLE_SWPROF11_POS    11
#define BLE_SWPROF10_BIT    ((uint32_t)0x00000400)
#define BLE_SWPROF10_POS    10
#define BLE_SWPROF9_BIT     ((uint32_t)0x00000200)
#define BLE_SWPROF9_POS     9
#define BLE_SWPROF8_BIT     ((uint32_t)0x00000100)
#define BLE_SWPROF8_POS     8
#define BLE_SWPROF7_BIT     ((uint32_t)0x00000080)
#define BLE_SWPROF7_POS     7
#define BLE_SWPROF6_BIT     ((uint32_t)0x00000040)
#define BLE_SWPROF6_POS     6
#define BLE_SWPROF5_BIT     ((uint32_t)0x00000020)
#define BLE_SWPROF5_POS     5
#define BLE_SWPROF4_BIT     ((uint32_t)0x00000010)
#define BLE_SWPROF4_POS     4
#define BLE_SWPROF3_BIT     ((uint32_t)0x00000008)
#define BLE_SWPROF3_POS     3
#define BLE_SWPROF2_BIT     ((uint32_t)0x00000004)
#define BLE_SWPROF2_POS     2
#define BLE_SWPROF1_BIT     ((uint32_t)0x00000002)
#define BLE_SWPROF1_POS     1
#define BLE_SWPROF0_BIT     ((uint32_t)0x00000001)
#define BLE_SWPROF0_POS     0

#define BLE_SWPROF31_RST    0x0
#define BLE_SWPROF30_RST    0x0
#define BLE_SWPROF29_RST    0x0
#define BLE_SWPROF28_RST    0x0
#define BLE_SWPROF27_RST    0x0
#define BLE_SWPROF26_RST    0x0
#define BLE_SWPROF25_RST    0x0
#define BLE_SWPROF24_RST    0x0
#define BLE_SWPROF23_RST    0x0
#define BLE_SWPROF22_RST    0x0
#define BLE_SWPROF21_RST    0x0
#define BLE_SWPROF20_RST    0x0
#define BLE_SWPROF19_RST    0x0
#define BLE_SWPROF18_RST    0x0
#define BLE_SWPROF17_RST    0x0
#define BLE_SWPROF16_RST    0x0
#define BLE_SWPROF15_RST    0x0
#define BLE_SWPROF14_RST    0x0
#define BLE_SWPROF13_RST    0x0
#define BLE_SWPROF12_RST    0x0
#define BLE_SWPROF11_RST    0x0
#define BLE_SWPROF10_RST    0x0
#define BLE_SWPROF9_RST     0x0
#define BLE_SWPROF8_RST     0x0
#define BLE_SWPROF7_RST     0x0
#define BLE_SWPROF6_RST     0x0
#define BLE_SWPROF5_RST     0x0
#define BLE_SWPROF4_RST     0x0
#define BLE_SWPROF3_RST     0x0
#define BLE_SWPROF2_RST     0x0
#define BLE_SWPROF1_RST     0x0
#define BLE_SWPROF0_RST     0x0

__INLINE void ble_swprofiling_pack(uint8_t swprof31, uint8_t swprof30, uint8_t swprof29, uint8_t swprof28, uint8_t swprof27, uint8_t swprof26, uint8_t swprof25, uint8_t swprof24, uint8_t swprof23, uint8_t swprof22, uint8_t swprof21, uint8_t swprof20, uint8_t swprof19, uint8_t swprof18, uint8_t swprof17, uint8_t swprof16, uint8_t swprof15, uint8_t swprof14, uint8_t swprof13, uint8_t swprof12, uint8_t swprof11, uint8_t swprof10, uint8_t swprof9, uint8_t swprof8, uint8_t swprof7, uint8_t swprof6, uint8_t swprof5, uint8_t swprof4, uint8_t swprof3, uint8_t swprof2, uint8_t swprof1, uint8_t swprof0)
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
    REG_BLE_WR(BLE_SWPROFILING_ADDR, ((uint32_t)swprof31 << 31) | ((uint32_t)swprof30 << 30) | ((uint32_t)swprof29 << 29) | ((uint32_t)swprof28 << 28) | ((uint32_t)swprof27 << 27) | ((uint32_t)swprof26 << 26) | ((uint32_t)swprof25 << 25) | ((uint32_t)swprof24 << 24) | ((uint32_t)swprof23 << 23) | ((uint32_t)swprof22 << 22) | ((uint32_t)swprof21 << 21) | ((uint32_t)swprof20 << 20) | ((uint32_t)swprof19 << 19) | ((uint32_t)swprof18 << 18) | ((uint32_t)swprof17 << 17) | ((uint32_t)swprof16 << 16) | ((uint32_t)swprof15 << 15) | ((uint32_t)swprof14 << 14) | ((uint32_t)swprof13 << 13) | ((uint32_t)swprof12 << 12) | ((uint32_t)swprof11 << 11) | ((uint32_t)swprof10 << 10) | ((uint32_t)swprof9 << 9) | ((uint32_t)swprof8 << 8) | ((uint32_t)swprof7 << 7) | ((uint32_t)swprof6 << 6) | ((uint32_t)swprof5 << 5) | ((uint32_t)swprof4 << 4) | ((uint32_t)swprof3 << 3) | ((uint32_t)swprof2 << 2) | ((uint32_t)swprof1 << 1) | ((uint32_t)swprof0 << 0));
}

__INLINE void ble_swprofiling_unpack(uint8_t *swprof31, uint8_t *swprof30, uint8_t *swprof29, uint8_t *swprof28, uint8_t *swprof27, uint8_t *swprof26, uint8_t *swprof25, uint8_t *swprof24, uint8_t *swprof23, uint8_t *swprof22, uint8_t *swprof21, uint8_t *swprof20, uint8_t *swprof19, uint8_t *swprof18, uint8_t *swprof17, uint8_t *swprof16, uint8_t *swprof15, uint8_t *swprof14, uint8_t *swprof13, uint8_t *swprof12, uint8_t *swprof11, uint8_t *swprof10, uint8_t *swprof9, uint8_t *swprof8, uint8_t *swprof7, uint8_t *swprof6, uint8_t *swprof5, uint8_t *swprof4, uint8_t *swprof3, uint8_t *swprof2, uint8_t *swprof1, uint8_t *swprof0)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);

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

__INLINE uint8_t ble_swprofiling_swprof31_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_swprofiling_swprof31_setf(uint8_t swprof31)
{
    ASSERT_ERR((((uint32_t)swprof31 << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)swprof31 << 31));
}

__INLINE uint8_t ble_swprofiling_swprof30_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ble_swprofiling_swprof30_setf(uint8_t swprof30)
{
    ASSERT_ERR((((uint32_t)swprof30 << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)swprof30 << 30));
}

__INLINE uint8_t ble_swprofiling_swprof29_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void ble_swprofiling_swprof29_setf(uint8_t swprof29)
{
    ASSERT_ERR((((uint32_t)swprof29 << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)swprof29 << 29));
}

__INLINE uint8_t ble_swprofiling_swprof28_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void ble_swprofiling_swprof28_setf(uint8_t swprof28)
{
    ASSERT_ERR((((uint32_t)swprof28 << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)swprof28 << 28));
}

__INLINE uint8_t ble_swprofiling_swprof27_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void ble_swprofiling_swprof27_setf(uint8_t swprof27)
{
    ASSERT_ERR((((uint32_t)swprof27 << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)swprof27 << 27));
}

__INLINE uint8_t ble_swprofiling_swprof26_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void ble_swprofiling_swprof26_setf(uint8_t swprof26)
{
    ASSERT_ERR((((uint32_t)swprof26 << 26) & ~((uint32_t)0x04000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)swprof26 << 26));
}

__INLINE uint8_t ble_swprofiling_swprof25_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void ble_swprofiling_swprof25_setf(uint8_t swprof25)
{
    ASSERT_ERR((((uint32_t)swprof25 << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)swprof25 << 25));
}

__INLINE uint8_t ble_swprofiling_swprof24_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void ble_swprofiling_swprof24_setf(uint8_t swprof24)
{
    ASSERT_ERR((((uint32_t)swprof24 << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)swprof24 << 24));
}

__INLINE uint8_t ble_swprofiling_swprof23_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void ble_swprofiling_swprof23_setf(uint8_t swprof23)
{
    ASSERT_ERR((((uint32_t)swprof23 << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)swprof23 << 23));
}

__INLINE uint8_t ble_swprofiling_swprof22_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void ble_swprofiling_swprof22_setf(uint8_t swprof22)
{
    ASSERT_ERR((((uint32_t)swprof22 << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)swprof22 << 22));
}

__INLINE uint8_t ble_swprofiling_swprof21_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void ble_swprofiling_swprof21_setf(uint8_t swprof21)
{
    ASSERT_ERR((((uint32_t)swprof21 << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)swprof21 << 21));
}

__INLINE uint8_t ble_swprofiling_swprof20_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void ble_swprofiling_swprof20_setf(uint8_t swprof20)
{
    ASSERT_ERR((((uint32_t)swprof20 << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)swprof20 << 20));
}

__INLINE uint8_t ble_swprofiling_swprof19_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void ble_swprofiling_swprof19_setf(uint8_t swprof19)
{
    ASSERT_ERR((((uint32_t)swprof19 << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)swprof19 << 19));
}

__INLINE uint8_t ble_swprofiling_swprof18_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void ble_swprofiling_swprof18_setf(uint8_t swprof18)
{
    ASSERT_ERR((((uint32_t)swprof18 << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)swprof18 << 18));
}

__INLINE uint8_t ble_swprofiling_swprof17_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void ble_swprofiling_swprof17_setf(uint8_t swprof17)
{
    ASSERT_ERR((((uint32_t)swprof17 << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)swprof17 << 17));
}

__INLINE uint8_t ble_swprofiling_swprof16_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void ble_swprofiling_swprof16_setf(uint8_t swprof16)
{
    ASSERT_ERR((((uint32_t)swprof16 << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)swprof16 << 16));
}

__INLINE uint8_t ble_swprofiling_swprof15_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ble_swprofiling_swprof15_setf(uint8_t swprof15)
{
    ASSERT_ERR((((uint32_t)swprof15 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)swprof15 << 15));
}

__INLINE uint8_t ble_swprofiling_swprof14_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ble_swprofiling_swprof14_setf(uint8_t swprof14)
{
    ASSERT_ERR((((uint32_t)swprof14 << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)swprof14 << 14));
}

__INLINE uint8_t ble_swprofiling_swprof13_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ble_swprofiling_swprof13_setf(uint8_t swprof13)
{
    ASSERT_ERR((((uint32_t)swprof13 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)swprof13 << 13));
}

__INLINE uint8_t ble_swprofiling_swprof12_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ble_swprofiling_swprof12_setf(uint8_t swprof12)
{
    ASSERT_ERR((((uint32_t)swprof12 << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)swprof12 << 12));
}

__INLINE uint8_t ble_swprofiling_swprof11_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ble_swprofiling_swprof11_setf(uint8_t swprof11)
{
    ASSERT_ERR((((uint32_t)swprof11 << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)swprof11 << 11));
}

__INLINE uint8_t ble_swprofiling_swprof10_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ble_swprofiling_swprof10_setf(uint8_t swprof10)
{
    ASSERT_ERR((((uint32_t)swprof10 << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)swprof10 << 10));
}

__INLINE uint8_t ble_swprofiling_swprof9_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ble_swprofiling_swprof9_setf(uint8_t swprof9)
{
    ASSERT_ERR((((uint32_t)swprof9 << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)swprof9 << 9));
}

__INLINE uint8_t ble_swprofiling_swprof8_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ble_swprofiling_swprof8_setf(uint8_t swprof8)
{
    ASSERT_ERR((((uint32_t)swprof8 << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)swprof8 << 8));
}

__INLINE uint8_t ble_swprofiling_swprof7_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ble_swprofiling_swprof7_setf(uint8_t swprof7)
{
    ASSERT_ERR((((uint32_t)swprof7 << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)swprof7 << 7));
}

__INLINE uint8_t ble_swprofiling_swprof6_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void ble_swprofiling_swprof6_setf(uint8_t swprof6)
{
    ASSERT_ERR((((uint32_t)swprof6 << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)swprof6 << 6));
}

__INLINE uint8_t ble_swprofiling_swprof5_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void ble_swprofiling_swprof5_setf(uint8_t swprof5)
{
    ASSERT_ERR((((uint32_t)swprof5 << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)swprof5 << 5));
}

__INLINE uint8_t ble_swprofiling_swprof4_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ble_swprofiling_swprof4_setf(uint8_t swprof4)
{
    ASSERT_ERR((((uint32_t)swprof4 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)swprof4 << 4));
}

__INLINE uint8_t ble_swprofiling_swprof3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_swprofiling_swprof3_setf(uint8_t swprof3)
{
    ASSERT_ERR((((uint32_t)swprof3 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)swprof3 << 3));
}

__INLINE uint8_t ble_swprofiling_swprof2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_swprofiling_swprof2_setf(uint8_t swprof2)
{
    ASSERT_ERR((((uint32_t)swprof2 << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)swprof2 << 2));
}

__INLINE uint8_t ble_swprofiling_swprof1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ble_swprofiling_swprof1_setf(uint8_t swprof1)
{
    ASSERT_ERR((((uint32_t)swprof1 << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)swprof1 << 1));
}

__INLINE uint8_t ble_swprofiling_swprof0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_swprofiling_swprof0_setf(uint8_t swprof0)
{
    ASSERT_ERR((((uint32_t)swprof0 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_SWPROFILING_ADDR, (REG_BLE_RD(BLE_SWPROFILING_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)swprof0 << 0));
}

/**
 * @brief RADIOCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16               SPIPTR   0x0
 *     07               SPICFG   0
 *  05:04              SPIFREQ   0x0
 *     01              SPICOMP   1
 *     00                SPIGO   0
 * </pre>
 */
#define BLE_RADIOCNTL0_ADDR   0x40100070
#define BLE_RADIOCNTL0_OFFSET 0x00000070
#define BLE_RADIOCNTL0_INDEX  0x0000001C
#define BLE_RADIOCNTL0_RESET  0x00000002

__INLINE uint32_t ble_radiocntl0_get(void)
{
    return REG_BLE_RD(BLE_RADIOCNTL0_ADDR);
}

__INLINE void ble_radiocntl0_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOCNTL0_ADDR, value);
}

// field definitions
#define BLE_SPIPTR_MASK    ((uint32_t)0xFFFF0000)
#define BLE_SPIPTR_LSB     16
#define BLE_SPIPTR_WIDTH   ((uint32_t)0x00000010)
#define BLE_SPICFG_BIT     ((uint32_t)0x00000080)
#define BLE_SPICFG_POS     7
#define BLE_SPIFREQ_MASK   ((uint32_t)0x00000030)
#define BLE_SPIFREQ_LSB    4
#define BLE_SPIFREQ_WIDTH  ((uint32_t)0x00000002)
#define BLE_SPICOMP_BIT    ((uint32_t)0x00000002)
#define BLE_SPICOMP_POS    1
#define BLE_SPIGO_BIT      ((uint32_t)0x00000001)
#define BLE_SPIGO_POS      0

#define BLE_SPIPTR_RST     0x0
#define BLE_SPICFG_RST     0x0
#define BLE_SPIFREQ_RST    0x0
#define BLE_SPICOMP_RST    0x1
#define BLE_SPIGO_RST      0x0

__INLINE void ble_radiocntl0_pack(uint16_t spiptr, uint8_t spicfg, uint8_t spifreq, uint8_t spigo)
{
    ASSERT_ERR((((uint32_t)spiptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)spicfg << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)spifreq << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)spigo << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL0_ADDR, ((uint32_t)spiptr << 16) | ((uint32_t)spicfg << 7) | ((uint32_t)spifreq << 4) | ((uint32_t)spigo << 0));
}

__INLINE void ble_radiocntl0_unpack(uint16_t *spiptr, uint8_t *spicfg, uint8_t *spifreq, uint8_t *spicomp, uint8_t *spigo)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL0_ADDR);

    *spiptr = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *spicfg = (localVal & ((uint32_t)0x00000080)) >> 7;
    *spifreq = (localVal & ((uint32_t)0x00000030)) >> 4;
    *spicomp = (localVal & ((uint32_t)0x00000002)) >> 1;
    *spigo = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t ble_radiocntl0_spiptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_radiocntl0_spiptr_setf(uint16_t spiptr)
{
    ASSERT_ERR((((uint32_t)spiptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL0_ADDR, (REG_BLE_RD(BLE_RADIOCNTL0_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)spiptr << 16));
}

__INLINE uint8_t ble_radiocntl0_spicfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ble_radiocntl0_spicfg_setf(uint8_t spicfg)
{
    ASSERT_ERR((((uint32_t)spicfg << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL0_ADDR, (REG_BLE_RD(BLE_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)spicfg << 7));
}

__INLINE uint8_t ble_radiocntl0_spifreq_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void ble_radiocntl0_spifreq_setf(uint8_t spifreq)
{
    ASSERT_ERR((((uint32_t)spifreq << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL0_ADDR, (REG_BLE_RD(BLE_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)spifreq << 4));
}

__INLINE uint8_t ble_radiocntl0_spicomp_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ble_radiocntl0_spigo_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_radiocntl0_spigo_setf(uint8_t spigo)
{
    ASSERT_ERR((((uint32_t)spigo << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL0_ADDR, (REG_BLE_RD(BLE_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)spigo << 0));
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
#define BLE_RADIOCNTL1_ADDR   0x40100074
#define BLE_RADIOCNTL1_OFFSET 0x00000074
#define BLE_RADIOCNTL1_INDEX  0x0000001D
#define BLE_RADIOCNTL1_RESET  0x00000000

__INLINE uint32_t ble_radiocntl1_get(void)
{
    return REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
}

__INLINE void ble_radiocntl1_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, value);
}

// field definitions
#define BLE_FORCEAGC_EN_BIT        ((uint32_t)0x80000000)
#define BLE_FORCEAGC_EN_POS        31
#define BLE_FORCEIQ_BIT            ((uint32_t)0x40000000)
#define BLE_FORCEIQ_POS            30
#define BLE_RXDNSL_BIT             ((uint32_t)0x20000000)
#define BLE_RXDNSL_POS             29
#define BLE_TXDNSL_BIT             ((uint32_t)0x10000000)
#define BLE_TXDNSL_POS             28
#define BLE_FORCEAGC_LENGTH_MASK   ((uint32_t)0x0FFF0000)
#define BLE_FORCEAGC_LENGTH_LSB    16
#define BLE_FORCEAGC_LENGTH_WIDTH  ((uint32_t)0x0000000C)
#define BLE_SYNC_PULSE_MODE_BIT    ((uint32_t)0x00008000)
#define BLE_SYNC_PULSE_MODE_POS    15
#define BLE_SYNC_PULSE_SRC_BIT     ((uint32_t)0x00004000)
#define BLE_SYNC_PULSE_SRC_POS     14
#define BLE_DPCORR_EN_BIT          ((uint32_t)0x00002000)
#define BLE_DPCORR_EN_POS          13
#define BLE_JEF_SELECT_BIT         ((uint32_t)0x00001000)
#define BLE_JEF_SELECT_POS         12
#define BLE_XRFSEL_MASK            ((uint32_t)0x000003F0)
#define BLE_XRFSEL_LSB             4
#define BLE_XRFSEL_WIDTH           ((uint32_t)0x00000006)
#define BLE_SUBVERSION_MASK        ((uint32_t)0x0000000F)
#define BLE_SUBVERSION_LSB         0
#define BLE_SUBVERSION_WIDTH       ((uint32_t)0x00000004)

#define BLE_FORCEAGC_EN_RST        0x0
#define BLE_FORCEIQ_RST            0x0
#define BLE_RXDNSL_RST             0x0
#define BLE_TXDNSL_RST             0x0
#define BLE_FORCEAGC_LENGTH_RST    0x0
#define BLE_SYNC_PULSE_MODE_RST    0x0
#define BLE_SYNC_PULSE_SRC_RST     0x0
#define BLE_DPCORR_EN_RST          0x0
#define BLE_JEF_SELECT_RST         0x0
#define BLE_XRFSEL_RST             0x0
#define BLE_SUBVERSION_RST         0x0

__INLINE void ble_radiocntl1_pack(uint8_t forceagcen, uint8_t forceiq, uint8_t rxdnsl, uint8_t txdnsl, uint16_t forceagclength, uint8_t syncpulsemode, uint8_t syncpulsesrc, uint8_t dpcorren, uint8_t jefselect, uint8_t xrfsel, uint8_t subversion)
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
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, ((uint32_t)forceagcen << 31) | ((uint32_t)forceiq << 30) | ((uint32_t)rxdnsl << 29) | ((uint32_t)txdnsl << 28) | ((uint32_t)forceagclength << 16) | ((uint32_t)syncpulsemode << 15) | ((uint32_t)syncpulsesrc << 14) | ((uint32_t)dpcorren << 13) | ((uint32_t)jefselect << 12) | ((uint32_t)xrfsel << 4) | ((uint32_t)subversion << 0));
}

__INLINE void ble_radiocntl1_unpack(uint8_t *forceagcen, uint8_t *forceiq, uint8_t *rxdnsl, uint8_t *txdnsl, uint16_t *forceagclength, uint8_t *syncpulsemode, uint8_t *syncpulsesrc, uint8_t *dpcorren, uint8_t *jefselect, uint8_t *xrfsel, uint8_t *subversion)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);

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

__INLINE uint8_t ble_radiocntl1_forceagc_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_radiocntl1_forceagc_en_setf(uint8_t forceagcen)
{
    ASSERT_ERR((((uint32_t)forceagcen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)forceagcen << 31));
}

__INLINE uint8_t ble_radiocntl1_forceiq_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ble_radiocntl1_forceiq_setf(uint8_t forceiq)
{
    ASSERT_ERR((((uint32_t)forceiq << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)forceiq << 30));
}

__INLINE uint8_t ble_radiocntl1_rxdnsl_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void ble_radiocntl1_rxdnsl_setf(uint8_t rxdnsl)
{
    ASSERT_ERR((((uint32_t)rxdnsl << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)rxdnsl << 29));
}

__INLINE uint8_t ble_radiocntl1_txdnsl_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void ble_radiocntl1_txdnsl_setf(uint8_t txdnsl)
{
    ASSERT_ERR((((uint32_t)txdnsl << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)txdnsl << 28));
}

__INLINE uint16_t ble_radiocntl1_forceagc_length_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void ble_radiocntl1_forceagc_length_setf(uint16_t forceagclength)
{
    ASSERT_ERR((((uint32_t)forceagclength << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)forceagclength << 16));
}

__INLINE uint8_t ble_radiocntl1_sync_pulse_mode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ble_radiocntl1_sync_pulse_mode_setf(uint8_t syncpulsemode)
{
    ASSERT_ERR((((uint32_t)syncpulsemode << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)syncpulsemode << 15));
}

__INLINE uint8_t ble_radiocntl1_sync_pulse_src_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ble_radiocntl1_sync_pulse_src_setf(uint8_t syncpulsesrc)
{
    ASSERT_ERR((((uint32_t)syncpulsesrc << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)syncpulsesrc << 14));
}

__INLINE uint8_t ble_radiocntl1_dpcorr_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ble_radiocntl1_dpcorr_en_setf(uint8_t dpcorren)
{
    ASSERT_ERR((((uint32_t)dpcorren << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)dpcorren << 13));
}

__INLINE uint8_t ble_radiocntl1_jef_select_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ble_radiocntl1_jef_select_setf(uint8_t jefselect)
{
    ASSERT_ERR((((uint32_t)jefselect << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)jefselect << 12));
}

__INLINE uint8_t ble_radiocntl1_xrfsel_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000003F0)) >> 4);
}

__INLINE void ble_radiocntl1_xrfsel_setf(uint8_t xrfsel)
{
    ASSERT_ERR((((uint32_t)xrfsel << 4) & ~((uint32_t)0x000003F0)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x000003F0)) | ((uint32_t)xrfsel << 4));
}

__INLINE uint8_t ble_radiocntl1_subversion_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ble_radiocntl1_subversion_setf(uint8_t subversion)
{
    ASSERT_ERR((((uint32_t)subversion << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, (REG_BLE_RD(BLE_RADIOCNTL1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)subversion << 0));
}

/**
 * @brief RADIOCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:30       LRSYNCCOMPMODE   0x3
 *     29       RXCITERMBYPASS   0
 *  28:24           LRVTBFLUSH   0x8
 *  23:22               PHYMSK   0x0
 *  21:20            LRSYNCERR   0x0
 *  18:16              SYNCERR   0x0
 *  15:00        FREQTABLE_PTR   0x100
 * </pre>
 */
#define BLE_RADIOCNTL2_ADDR   0x40100078
#define BLE_RADIOCNTL2_OFFSET 0x00000078
#define BLE_RADIOCNTL2_INDEX  0x0000001E
#define BLE_RADIOCNTL2_RESET  0xC8000100

__INLINE uint32_t ble_radiocntl2_get(void)
{
    return REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
}

__INLINE void ble_radiocntl2_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, value);
}

// field definitions
#define BLE_LRSYNCCOMPMODE_MASK   ((uint32_t)0xC0000000)
#define BLE_LRSYNCCOMPMODE_LSB    30
#define BLE_LRSYNCCOMPMODE_WIDTH  ((uint32_t)0x00000002)
#define BLE_RXCITERMBYPASS_BIT    ((uint32_t)0x20000000)
#define BLE_RXCITERMBYPASS_POS    29
#define BLE_LRVTBFLUSH_MASK       ((uint32_t)0x1F000000)
#define BLE_LRVTBFLUSH_LSB        24
#define BLE_LRVTBFLUSH_WIDTH      ((uint32_t)0x00000005)
#define BLE_PHYMSK_MASK           ((uint32_t)0x00C00000)
#define BLE_PHYMSK_LSB            22
#define BLE_PHYMSK_WIDTH          ((uint32_t)0x00000002)
#define BLE_LRSYNCERR_MASK        ((uint32_t)0x00300000)
#define BLE_LRSYNCERR_LSB         20
#define BLE_LRSYNCERR_WIDTH       ((uint32_t)0x00000002)
#define BLE_SYNCERR_MASK          ((uint32_t)0x00070000)
#define BLE_SYNCERR_LSB           16
#define BLE_SYNCERR_WIDTH         ((uint32_t)0x00000003)
#define BLE_FREQTABLE_PTR_MASK    ((uint32_t)0x0000FFFF)
#define BLE_FREQTABLE_PTR_LSB     0
#define BLE_FREQTABLE_PTR_WIDTH   ((uint32_t)0x00000010)

#define BLE_LRSYNCCOMPMODE_RST    0x3
#define BLE_RXCITERMBYPASS_RST    0x0
#define BLE_LRVTBFLUSH_RST        0x8
#define BLE_PHYMSK_RST            0x0
#define BLE_LRSYNCERR_RST         0x0
#define BLE_SYNCERR_RST           0x0
#define BLE_FREQTABLE_PTR_RST     0x100

__INLINE void ble_radiocntl2_pack(uint8_t lrsynccompmode, uint8_t rxcitermbypass, uint8_t lrvtbflush, uint8_t phymsk, uint8_t lrsyncerr, uint8_t syncerr, uint16_t freqtableptr)
{
    ASSERT_ERR((((uint32_t)lrsynccompmode << 30) & ~((uint32_t)0xC0000000)) == 0);
    ASSERT_ERR((((uint32_t)rxcitermbypass << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)lrvtbflush << 24) & ~((uint32_t)0x1F000000)) == 0);
    ASSERT_ERR((((uint32_t)phymsk << 22) & ~((uint32_t)0x00C00000)) == 0);
    ASSERT_ERR((((uint32_t)lrsyncerr << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)syncerr << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)freqtableptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, ((uint32_t)lrsynccompmode << 30) | ((uint32_t)rxcitermbypass << 29) | ((uint32_t)lrvtbflush << 24) | ((uint32_t)phymsk << 22) | ((uint32_t)lrsyncerr << 20) | ((uint32_t)syncerr << 16) | ((uint32_t)freqtableptr << 0));
}

__INLINE void ble_radiocntl2_unpack(uint8_t *lrsynccompmode, uint8_t *rxcitermbypass, uint8_t *lrvtbflush, uint8_t *phymsk, uint8_t *lrsyncerr, uint8_t *syncerr, uint16_t *freqtableptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);

    *lrsynccompmode = (localVal & ((uint32_t)0xC0000000)) >> 30;
    *rxcitermbypass = (localVal & ((uint32_t)0x20000000)) >> 29;
    *lrvtbflush = (localVal & ((uint32_t)0x1F000000)) >> 24;
    *phymsk = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *lrsyncerr = (localVal & ((uint32_t)0x00300000)) >> 20;
    *syncerr = (localVal & ((uint32_t)0x00070000)) >> 16;
    *freqtableptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t ble_radiocntl2_lrsynccompmode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0xC0000000)) >> 30);
}

__INLINE void ble_radiocntl2_lrsynccompmode_setf(uint8_t lrsynccompmode)
{
    ASSERT_ERR((((uint32_t)lrsynccompmode << 30) & ~((uint32_t)0xC0000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, (REG_BLE_RD(BLE_RADIOCNTL2_ADDR) & ~((uint32_t)0xC0000000)) | ((uint32_t)lrsynccompmode << 30));
}

__INLINE uint8_t ble_radiocntl2_rxcitermbypass_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void ble_radiocntl2_rxcitermbypass_setf(uint8_t rxcitermbypass)
{
    ASSERT_ERR((((uint32_t)rxcitermbypass << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, (REG_BLE_RD(BLE_RADIOCNTL2_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)rxcitermbypass << 29));
}

__INLINE uint8_t ble_radiocntl2_lrvtbflush_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x1F000000)) >> 24);
}

__INLINE void ble_radiocntl2_lrvtbflush_setf(uint8_t lrvtbflush)
{
    ASSERT_ERR((((uint32_t)lrvtbflush << 24) & ~((uint32_t)0x1F000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, (REG_BLE_RD(BLE_RADIOCNTL2_ADDR) & ~((uint32_t)0x1F000000)) | ((uint32_t)lrvtbflush << 24));
}

__INLINE uint8_t ble_radiocntl2_phymsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE void ble_radiocntl2_phymsk_setf(uint8_t phymsk)
{
    ASSERT_ERR((((uint32_t)phymsk << 22) & ~((uint32_t)0x00C00000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, (REG_BLE_RD(BLE_RADIOCNTL2_ADDR) & ~((uint32_t)0x00C00000)) | ((uint32_t)phymsk << 22));
}

__INLINE uint8_t ble_radiocntl2_lrsyncerr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void ble_radiocntl2_lrsyncerr_setf(uint8_t lrsyncerr)
{
    ASSERT_ERR((((uint32_t)lrsyncerr << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, (REG_BLE_RD(BLE_RADIOCNTL2_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)lrsyncerr << 20));
}

__INLINE uint8_t ble_radiocntl2_syncerr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void ble_radiocntl2_syncerr_setf(uint8_t syncerr)
{
    ASSERT_ERR((((uint32_t)syncerr << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, (REG_BLE_RD(BLE_RADIOCNTL2_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)syncerr << 16));
}

__INLINE uint16_t ble_radiocntl2_freqtable_ptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_radiocntl2_freqtable_ptr_setf(uint16_t freqtableptr)
{
    ASSERT_ERR((((uint32_t)freqtableptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL2_ADDR, (REG_BLE_RD(BLE_RADIOCNTL2_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)freqtableptr << 0));
}

/**
 * @brief RADIOCNTL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:30           RXRATE3CFG   0x3
 *  29:28           RXRATE2CFG   0x2
 *  27:26           RXRATE1CFG   0x1
 *  25:24           RXRATE0CFG   0x0
 *     18       RXSYNC_ROUTING   0
 *  17:16          RXVALID_BEH   0x0
 *  15:14           TXRATE3CFG   0x3
 *  13:12           TXRATE2CFG   0x2
 *  11:10           TXRATE1CFG   0x1
 *  09:08           TXRATE0CFG   0x0
 *  01:00          TXVALID_BEH   0x0
 * </pre>
 */
#define BLE_RADIOCNTL3_ADDR   0x4010007C
#define BLE_RADIOCNTL3_OFFSET 0x0000007C
#define BLE_RADIOCNTL3_INDEX  0x0000001F
#define BLE_RADIOCNTL3_RESET  0xE400E400

__INLINE uint32_t ble_radiocntl3_get(void)
{
    return REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
}

__INLINE void ble_radiocntl3_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, value);
}

// field definitions
#define BLE_RXRATE3CFG_MASK       ((uint32_t)0xC0000000)
#define BLE_RXRATE3CFG_LSB        30
#define BLE_RXRATE3CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_RXRATE2CFG_MASK       ((uint32_t)0x30000000)
#define BLE_RXRATE2CFG_LSB        28
#define BLE_RXRATE2CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_RXRATE1CFG_MASK       ((uint32_t)0x0C000000)
#define BLE_RXRATE1CFG_LSB        26
#define BLE_RXRATE1CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_RXRATE0CFG_MASK       ((uint32_t)0x03000000)
#define BLE_RXRATE0CFG_LSB        24
#define BLE_RXRATE0CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_RXSYNC_ROUTING_BIT    ((uint32_t)0x00040000)
#define BLE_RXSYNC_ROUTING_POS    18
#define BLE_RXVALID_BEH_MASK      ((uint32_t)0x00030000)
#define BLE_RXVALID_BEH_LSB       16
#define BLE_RXVALID_BEH_WIDTH     ((uint32_t)0x00000002)
#define BLE_TXRATE3CFG_MASK       ((uint32_t)0x0000C000)
#define BLE_TXRATE3CFG_LSB        14
#define BLE_TXRATE3CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_TXRATE2CFG_MASK       ((uint32_t)0x00003000)
#define BLE_TXRATE2CFG_LSB        12
#define BLE_TXRATE2CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_TXRATE1CFG_MASK       ((uint32_t)0x00000C00)
#define BLE_TXRATE1CFG_LSB        10
#define BLE_TXRATE1CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_TXRATE0CFG_MASK       ((uint32_t)0x00000300)
#define BLE_TXRATE0CFG_LSB        8
#define BLE_TXRATE0CFG_WIDTH      ((uint32_t)0x00000002)
#define BLE_TXVALID_BEH_MASK      ((uint32_t)0x00000003)
#define BLE_TXVALID_BEH_LSB       0
#define BLE_TXVALID_BEH_WIDTH     ((uint32_t)0x00000002)

#define BLE_RXRATE3CFG_RST        0x3
#define BLE_RXRATE2CFG_RST        0x2
#define BLE_RXRATE1CFG_RST        0x1
#define BLE_RXRATE0CFG_RST        0x0
#define BLE_RXSYNC_ROUTING_RST    0x0
#define BLE_RXVALID_BEH_RST       0x0
#define BLE_TXRATE3CFG_RST        0x3
#define BLE_TXRATE2CFG_RST        0x2
#define BLE_TXRATE1CFG_RST        0x1
#define BLE_TXRATE0CFG_RST        0x0
#define BLE_TXVALID_BEH_RST       0x0

__INLINE void ble_radiocntl3_pack(uint8_t rxrate3cfg, uint8_t rxrate2cfg, uint8_t rxrate1cfg, uint8_t rxrate0cfg, uint8_t rxsyncrouting, uint8_t rxvalidbeh, uint8_t txrate3cfg, uint8_t txrate2cfg, uint8_t txrate1cfg, uint8_t txrate0cfg, uint8_t txvalidbeh)
{
    ASSERT_ERR((((uint32_t)rxrate3cfg << 30) & ~((uint32_t)0xC0000000)) == 0);
    ASSERT_ERR((((uint32_t)rxrate2cfg << 28) & ~((uint32_t)0x30000000)) == 0);
    ASSERT_ERR((((uint32_t)rxrate1cfg << 26) & ~((uint32_t)0x0C000000)) == 0);
    ASSERT_ERR((((uint32_t)rxrate0cfg << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)rxsyncrouting << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)rxvalidbeh << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)txrate3cfg << 14) & ~((uint32_t)0x0000C000)) == 0);
    ASSERT_ERR((((uint32_t)txrate2cfg << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)txrate1cfg << 10) & ~((uint32_t)0x00000C00)) == 0);
    ASSERT_ERR((((uint32_t)txrate0cfg << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)txvalidbeh << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, ((uint32_t)rxrate3cfg << 30) | ((uint32_t)rxrate2cfg << 28) | ((uint32_t)rxrate1cfg << 26) | ((uint32_t)rxrate0cfg << 24) | ((uint32_t)rxsyncrouting << 18) | ((uint32_t)rxvalidbeh << 16) | ((uint32_t)txrate3cfg << 14) | ((uint32_t)txrate2cfg << 12) | ((uint32_t)txrate1cfg << 10) | ((uint32_t)txrate0cfg << 8) | ((uint32_t)txvalidbeh << 0));
}

__INLINE void ble_radiocntl3_unpack(uint8_t *rxrate3cfg, uint8_t *rxrate2cfg, uint8_t *rxrate1cfg, uint8_t *rxrate0cfg, uint8_t *rxsyncrouting, uint8_t *rxvalidbeh, uint8_t *txrate3cfg, uint8_t *txrate2cfg, uint8_t *txrate1cfg, uint8_t *txrate0cfg, uint8_t *txvalidbeh)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);

    *rxrate3cfg = (localVal & ((uint32_t)0xC0000000)) >> 30;
    *rxrate2cfg = (localVal & ((uint32_t)0x30000000)) >> 28;
    *rxrate1cfg = (localVal & ((uint32_t)0x0C000000)) >> 26;
    *rxrate0cfg = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rxsyncrouting = (localVal & ((uint32_t)0x00040000)) >> 18;
    *rxvalidbeh = (localVal & ((uint32_t)0x00030000)) >> 16;
    *txrate3cfg = (localVal & ((uint32_t)0x0000C000)) >> 14;
    *txrate2cfg = (localVal & ((uint32_t)0x00003000)) >> 12;
    *txrate1cfg = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *txrate0cfg = (localVal & ((uint32_t)0x00000300)) >> 8;
    *txvalidbeh = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t ble_radiocntl3_rxrate3cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0xC0000000)) >> 30);
}

__INLINE void ble_radiocntl3_rxrate3cfg_setf(uint8_t rxrate3cfg)
{
    ASSERT_ERR((((uint32_t)rxrate3cfg << 30) & ~((uint32_t)0xC0000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0xC0000000)) | ((uint32_t)rxrate3cfg << 30));
}

__INLINE uint8_t ble_radiocntl3_rxrate2cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void ble_radiocntl3_rxrate2cfg_setf(uint8_t rxrate2cfg)
{
    ASSERT_ERR((((uint32_t)rxrate2cfg << 28) & ~((uint32_t)0x30000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)rxrate2cfg << 28));
}

__INLINE uint8_t ble_radiocntl3_rxrate1cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x0C000000)) >> 26);
}

__INLINE void ble_radiocntl3_rxrate1cfg_setf(uint8_t rxrate1cfg)
{
    ASSERT_ERR((((uint32_t)rxrate1cfg << 26) & ~((uint32_t)0x0C000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x0C000000)) | ((uint32_t)rxrate1cfg << 26));
}

__INLINE uint8_t ble_radiocntl3_rxrate0cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void ble_radiocntl3_rxrate0cfg_setf(uint8_t rxrate0cfg)
{
    ASSERT_ERR((((uint32_t)rxrate0cfg << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rxrate0cfg << 24));
}

__INLINE uint8_t ble_radiocntl3_rxsync_routing_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void ble_radiocntl3_rxsync_routing_setf(uint8_t rxsyncrouting)
{
    ASSERT_ERR((((uint32_t)rxsyncrouting << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)rxsyncrouting << 18));
}

__INLINE uint8_t ble_radiocntl3_rxvalid_beh_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void ble_radiocntl3_rxvalid_beh_setf(uint8_t rxvalidbeh)
{
    ASSERT_ERR((((uint32_t)rxvalidbeh << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)rxvalidbeh << 16));
}

__INLINE uint8_t ble_radiocntl3_txrate3cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void ble_radiocntl3_txrate3cfg_setf(uint8_t txrate3cfg)
{
    ASSERT_ERR((((uint32_t)txrate3cfg << 14) & ~((uint32_t)0x0000C000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)txrate3cfg << 14));
}

__INLINE uint8_t ble_radiocntl3_txrate2cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void ble_radiocntl3_txrate2cfg_setf(uint8_t txrate2cfg)
{
    ASSERT_ERR((((uint32_t)txrate2cfg << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)txrate2cfg << 12));
}

__INLINE uint8_t ble_radiocntl3_txrate1cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void ble_radiocntl3_txrate1cfg_setf(uint8_t txrate1cfg)
{
    ASSERT_ERR((((uint32_t)txrate1cfg << 10) & ~((uint32_t)0x00000C00)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)txrate1cfg << 10));
}

__INLINE uint8_t ble_radiocntl3_txrate0cfg_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void ble_radiocntl3_txrate0cfg_setf(uint8_t txrate0cfg)
{
    ASSERT_ERR((((uint32_t)txrate0cfg << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)txrate0cfg << 8));
}

__INLINE uint8_t ble_radiocntl3_txvalid_beh_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOCNTL3_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void ble_radiocntl3_txvalid_beh_setf(uint8_t txvalidbeh)
{
    ASSERT_ERR((((uint32_t)txvalidbeh << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_RADIOCNTL3_ADDR, (REG_BLE_RD(BLE_RADIOCNTL3_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)txvalidbeh << 0));
}

/**
 * @brief RADIOPWRUPDN0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24       SYNC_POSITION0   0x0
 *  23:16             RXPWRUP0   0x0
 *  14:08             TXPWRDN0   0x0
 *  07:00             TXPWRUP0   0x0
 * </pre>
 */
#define BLE_RADIOPWRUPDN0_ADDR   0x40100080
#define BLE_RADIOPWRUPDN0_OFFSET 0x00000080
#define BLE_RADIOPWRUPDN0_INDEX  0x00000020
#define BLE_RADIOPWRUPDN0_RESET  0x00000000

__INLINE uint32_t ble_radiopwrupdn0_get(void)
{
    return REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR);
}

__INLINE void ble_radiopwrupdn0_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOPWRUPDN0_ADDR, value);
}

// field definitions
#define BLE_SYNC_POSITION0_MASK   ((uint32_t)0xFF000000)
#define BLE_SYNC_POSITION0_LSB    24
#define BLE_SYNC_POSITION0_WIDTH  ((uint32_t)0x00000008)
#define BLE_RXPWRUP0_MASK         ((uint32_t)0x00FF0000)
#define BLE_RXPWRUP0_LSB          16
#define BLE_RXPWRUP0_WIDTH        ((uint32_t)0x00000008)
#define BLE_TXPWRDN0_MASK         ((uint32_t)0x00007F00)
#define BLE_TXPWRDN0_LSB          8
#define BLE_TXPWRDN0_WIDTH        ((uint32_t)0x00000007)
#define BLE_TXPWRUP0_MASK         ((uint32_t)0x000000FF)
#define BLE_TXPWRUP0_LSB          0
#define BLE_TXPWRUP0_WIDTH        ((uint32_t)0x00000008)

#define BLE_SYNC_POSITION0_RST    0x0
#define BLE_RXPWRUP0_RST          0x0
#define BLE_TXPWRDN0_RST          0x0
#define BLE_TXPWRUP0_RST          0x0

__INLINE void ble_radiopwrupdn0_pack(uint8_t syncposition0, uint8_t rxpwrup0, uint8_t txpwrdn0, uint8_t txpwrup0)
{
    ASSERT_ERR((((uint32_t)syncposition0 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rxpwrup0 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)txpwrdn0 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpwrup0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN0_ADDR, ((uint32_t)syncposition0 << 24) | ((uint32_t)rxpwrup0 << 16) | ((uint32_t)txpwrdn0 << 8) | ((uint32_t)txpwrup0 << 0));
}

__INLINE void ble_radiopwrupdn0_unpack(uint8_t *syncposition0, uint8_t *rxpwrup0, uint8_t *txpwrdn0, uint8_t *txpwrup0)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR);

    *syncposition0 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rxpwrup0 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txpwrdn0 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpwrup0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_radiopwrupdn0_sync_position0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void ble_radiopwrupdn0_sync_position0_setf(uint8_t syncposition0)
{
    ASSERT_ERR((((uint32_t)syncposition0 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN0_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)syncposition0 << 24));
}

__INLINE uint8_t ble_radiopwrupdn0_rxpwrup0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_radiopwrupdn0_rxpwrup0_setf(uint8_t rxpwrup0)
{
    ASSERT_ERR((((uint32_t)rxpwrup0 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN0_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxpwrup0 << 16));
}

__INLINE uint8_t ble_radiopwrupdn0_txpwrdn0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_radiopwrupdn0_txpwrdn0_setf(uint8_t txpwrdn0)
{
    ASSERT_ERR((((uint32_t)txpwrdn0 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN0_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)txpwrdn0 << 8));
}

__INLINE uint8_t ble_radiopwrupdn0_txpwrup0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_radiopwrupdn0_txpwrup0_setf(uint8_t txpwrup0)
{
    ASSERT_ERR((((uint32_t)txpwrup0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN0_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN0_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txpwrup0 << 0));
}

/**
 * @brief RADIOPWRUPDN1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24       SYNC_POSITION1   0x0
 *  23:16             RXPWRUP1   0x0
 *  14:08             TXPWRDN1   0x0
 *  07:00             TXPWRUP1   0x0
 * </pre>
 */
#define BLE_RADIOPWRUPDN1_ADDR   0x40100084
#define BLE_RADIOPWRUPDN1_OFFSET 0x00000084
#define BLE_RADIOPWRUPDN1_INDEX  0x00000021
#define BLE_RADIOPWRUPDN1_RESET  0x00000000

__INLINE uint32_t ble_radiopwrupdn1_get(void)
{
    return REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR);
}

__INLINE void ble_radiopwrupdn1_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOPWRUPDN1_ADDR, value);
}

// field definitions
#define BLE_SYNC_POSITION1_MASK   ((uint32_t)0xFF000000)
#define BLE_SYNC_POSITION1_LSB    24
#define BLE_SYNC_POSITION1_WIDTH  ((uint32_t)0x00000008)
#define BLE_RXPWRUP1_MASK         ((uint32_t)0x00FF0000)
#define BLE_RXPWRUP1_LSB          16
#define BLE_RXPWRUP1_WIDTH        ((uint32_t)0x00000008)
#define BLE_TXPWRDN1_MASK         ((uint32_t)0x00007F00)
#define BLE_TXPWRDN1_LSB          8
#define BLE_TXPWRDN1_WIDTH        ((uint32_t)0x00000007)
#define BLE_TXPWRUP1_MASK         ((uint32_t)0x000000FF)
#define BLE_TXPWRUP1_LSB          0
#define BLE_TXPWRUP1_WIDTH        ((uint32_t)0x00000008)

#define BLE_SYNC_POSITION1_RST    0x0
#define BLE_RXPWRUP1_RST          0x0
#define BLE_TXPWRDN1_RST          0x0
#define BLE_TXPWRUP1_RST          0x0

__INLINE void ble_radiopwrupdn1_pack(uint8_t syncposition1, uint8_t rxpwrup1, uint8_t txpwrdn1, uint8_t txpwrup1)
{
    ASSERT_ERR((((uint32_t)syncposition1 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rxpwrup1 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)txpwrdn1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpwrup1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN1_ADDR, ((uint32_t)syncposition1 << 24) | ((uint32_t)rxpwrup1 << 16) | ((uint32_t)txpwrdn1 << 8) | ((uint32_t)txpwrup1 << 0));
}

__INLINE void ble_radiopwrupdn1_unpack(uint8_t *syncposition1, uint8_t *rxpwrup1, uint8_t *txpwrdn1, uint8_t *txpwrup1)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR);

    *syncposition1 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rxpwrup1 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txpwrdn1 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpwrup1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_radiopwrupdn1_sync_position1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void ble_radiopwrupdn1_sync_position1_setf(uint8_t syncposition1)
{
    ASSERT_ERR((((uint32_t)syncposition1 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN1_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)syncposition1 << 24));
}

__INLINE uint8_t ble_radiopwrupdn1_rxpwrup1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_radiopwrupdn1_rxpwrup1_setf(uint8_t rxpwrup1)
{
    ASSERT_ERR((((uint32_t)rxpwrup1 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN1_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxpwrup1 << 16));
}

__INLINE uint8_t ble_radiopwrupdn1_txpwrdn1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_radiopwrupdn1_txpwrdn1_setf(uint8_t txpwrdn1)
{
    ASSERT_ERR((((uint32_t)txpwrdn1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN1_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)txpwrdn1 << 8));
}

__INLINE uint8_t ble_radiopwrupdn1_txpwrup1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_radiopwrupdn1_txpwrup1_setf(uint8_t txpwrup1)
{
    ASSERT_ERR((((uint32_t)txpwrup1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN1_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN1_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txpwrup1 << 0));
}

/**
 * @brief RADIOPWRUPDN2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24       SYNC_POSITION2   0x0
 *  23:16             RXPWRUP2   0x0
 *  14:08             TXPWRDN2   0x0
 *  07:00             TXPWRUP2   0x0
 * </pre>
 */
#define BLE_RADIOPWRUPDN2_ADDR   0x40100088
#define BLE_RADIOPWRUPDN2_OFFSET 0x00000088
#define BLE_RADIOPWRUPDN2_INDEX  0x00000022
#define BLE_RADIOPWRUPDN2_RESET  0x00000000

__INLINE uint32_t ble_radiopwrupdn2_get(void)
{
    return REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR);
}

__INLINE void ble_radiopwrupdn2_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOPWRUPDN2_ADDR, value);
}

// field definitions
#define BLE_SYNC_POSITION2_MASK   ((uint32_t)0xFF000000)
#define BLE_SYNC_POSITION2_LSB    24
#define BLE_SYNC_POSITION2_WIDTH  ((uint32_t)0x00000008)
#define BLE_RXPWRUP2_MASK         ((uint32_t)0x00FF0000)
#define BLE_RXPWRUP2_LSB          16
#define BLE_RXPWRUP2_WIDTH        ((uint32_t)0x00000008)
#define BLE_TXPWRDN2_MASK         ((uint32_t)0x00007F00)
#define BLE_TXPWRDN2_LSB          8
#define BLE_TXPWRDN2_WIDTH        ((uint32_t)0x00000007)
#define BLE_TXPWRUP2_MASK         ((uint32_t)0x000000FF)
#define BLE_TXPWRUP2_LSB          0
#define BLE_TXPWRUP2_WIDTH        ((uint32_t)0x00000008)

#define BLE_SYNC_POSITION2_RST    0x0
#define BLE_RXPWRUP2_RST          0x0
#define BLE_TXPWRDN2_RST          0x0
#define BLE_TXPWRUP2_RST          0x0

__INLINE void ble_radiopwrupdn2_pack(uint8_t syncposition2, uint8_t rxpwrup2, uint8_t txpwrdn2, uint8_t txpwrup2)
{
    ASSERT_ERR((((uint32_t)syncposition2 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rxpwrup2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)txpwrdn2 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpwrup2 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN2_ADDR, ((uint32_t)syncposition2 << 24) | ((uint32_t)rxpwrup2 << 16) | ((uint32_t)txpwrdn2 << 8) | ((uint32_t)txpwrup2 << 0));
}

__INLINE void ble_radiopwrupdn2_unpack(uint8_t *syncposition2, uint8_t *rxpwrup2, uint8_t *txpwrdn2, uint8_t *txpwrup2)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR);

    *syncposition2 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rxpwrup2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txpwrdn2 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpwrup2 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_radiopwrupdn2_sync_position2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void ble_radiopwrupdn2_sync_position2_setf(uint8_t syncposition2)
{
    ASSERT_ERR((((uint32_t)syncposition2 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN2_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)syncposition2 << 24));
}

__INLINE uint8_t ble_radiopwrupdn2_rxpwrup2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_radiopwrupdn2_rxpwrup2_setf(uint8_t rxpwrup2)
{
    ASSERT_ERR((((uint32_t)rxpwrup2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN2_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxpwrup2 << 16));
}

__INLINE uint8_t ble_radiopwrupdn2_txpwrdn2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_radiopwrupdn2_txpwrdn2_setf(uint8_t txpwrdn2)
{
    ASSERT_ERR((((uint32_t)txpwrdn2 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN2_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)txpwrdn2 << 8));
}

__INLINE uint8_t ble_radiopwrupdn2_txpwrup2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_radiopwrupdn2_txpwrup2_setf(uint8_t txpwrup2)
{
    ASSERT_ERR((((uint32_t)txpwrup2 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN2_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN2_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txpwrup2 << 0));
}

/**
 * @brief RADIOPWRUPDN3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  14:08             TXPWRDN3   0x0
 *  07:00             TXPWRUP3   0x0
 * </pre>
 */
#define BLE_RADIOPWRUPDN3_ADDR   0x4010008C
#define BLE_RADIOPWRUPDN3_OFFSET 0x0000008C
#define BLE_RADIOPWRUPDN3_INDEX  0x00000023
#define BLE_RADIOPWRUPDN3_RESET  0x00000000

__INLINE uint32_t ble_radiopwrupdn3_get(void)
{
    return REG_BLE_RD(BLE_RADIOPWRUPDN3_ADDR);
}

__INLINE void ble_radiopwrupdn3_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOPWRUPDN3_ADDR, value);
}

// field definitions
#define BLE_TXPWRDN3_MASK   ((uint32_t)0x00007F00)
#define BLE_TXPWRDN3_LSB    8
#define BLE_TXPWRDN3_WIDTH  ((uint32_t)0x00000007)
#define BLE_TXPWRUP3_MASK   ((uint32_t)0x000000FF)
#define BLE_TXPWRUP3_LSB    0
#define BLE_TXPWRUP3_WIDTH  ((uint32_t)0x00000008)

#define BLE_TXPWRDN3_RST    0x0
#define BLE_TXPWRUP3_RST    0x0

__INLINE void ble_radiopwrupdn3_pack(uint8_t txpwrdn3, uint8_t txpwrup3)
{
    ASSERT_ERR((((uint32_t)txpwrdn3 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpwrup3 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN3_ADDR, ((uint32_t)txpwrdn3 << 8) | ((uint32_t)txpwrup3 << 0));
}

__INLINE void ble_radiopwrupdn3_unpack(uint8_t *txpwrdn3, uint8_t *txpwrup3)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN3_ADDR);

    *txpwrdn3 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpwrup3 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_radiopwrupdn3_txpwrdn3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN3_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_radiopwrupdn3_txpwrdn3_setf(uint8_t txpwrdn3)
{
    ASSERT_ERR((((uint32_t)txpwrdn3 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN3_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN3_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)txpwrdn3 << 8));
}

__INLINE uint8_t ble_radiopwrupdn3_txpwrup3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOPWRUPDN3_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_radiopwrupdn3_txpwrup3_setf(uint8_t txpwrup3)
{
    ASSERT_ERR((((uint32_t)txpwrup3 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_RADIOPWRUPDN3_ADDR, (REG_BLE_RD(BLE_RADIOPWRUPDN3_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txpwrup3 << 0));
}

/**
 * @brief RADIOTXRXTIM0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:16            RFRXTMDA0   0x0
 *  14:08           RXPATHDLY0   0x0
 *  06:00           TXPATHDLY0   0x0
 * </pre>
 */
#define BLE_RADIOTXRXTIM0_ADDR   0x40100090
#define BLE_RADIOTXRXTIM0_OFFSET 0x00000090
#define BLE_RADIOTXRXTIM0_INDEX  0x00000024
#define BLE_RADIOTXRXTIM0_RESET  0x00000000

__INLINE uint32_t ble_radiotxrxtim0_get(void)
{
    return REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR);
}

__INLINE void ble_radiotxrxtim0_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOTXRXTIM0_ADDR, value);
}

// field definitions
#define BLE_RFRXTMDA0_MASK    ((uint32_t)0x007F0000)
#define BLE_RFRXTMDA0_LSB     16
#define BLE_RFRXTMDA0_WIDTH   ((uint32_t)0x00000007)
#define BLE_RXPATHDLY0_MASK   ((uint32_t)0x00007F00)
#define BLE_RXPATHDLY0_LSB    8
#define BLE_RXPATHDLY0_WIDTH  ((uint32_t)0x00000007)
#define BLE_TXPATHDLY0_MASK   ((uint32_t)0x0000007F)
#define BLE_TXPATHDLY0_LSB    0
#define BLE_TXPATHDLY0_WIDTH  ((uint32_t)0x00000007)

#define BLE_RFRXTMDA0_RST     0x0
#define BLE_RXPATHDLY0_RST    0x0
#define BLE_TXPATHDLY0_RST    0x0

__INLINE void ble_radiotxrxtim0_pack(uint8_t rfrxtmda0, uint8_t rxpathdly0, uint8_t txpathdly0)
{
    ASSERT_ERR((((uint32_t)rfrxtmda0 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxpathdly0 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpathdly0 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM0_ADDR, ((uint32_t)rfrxtmda0 << 16) | ((uint32_t)rxpathdly0 << 8) | ((uint32_t)txpathdly0 << 0));
}

__INLINE void ble_radiotxrxtim0_unpack(uint8_t *rfrxtmda0, uint8_t *rxpathdly0, uint8_t *txpathdly0)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR);

    *rfrxtmda0 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *rxpathdly0 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpathdly0 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t ble_radiotxrxtim0_rfrxtmda0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void ble_radiotxrxtim0_rfrxtmda0_setf(uint8_t rfrxtmda0)
{
    ASSERT_ERR((((uint32_t)rfrxtmda0 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM0_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)rfrxtmda0 << 16));
}

__INLINE uint8_t ble_radiotxrxtim0_rxpathdly0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_radiotxrxtim0_rxpathdly0_setf(uint8_t rxpathdly0)
{
    ASSERT_ERR((((uint32_t)rxpathdly0 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM0_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)rxpathdly0 << 8));
}

__INLINE uint8_t ble_radiotxrxtim0_txpathdly0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void ble_radiotxrxtim0_txpathdly0_setf(uint8_t txpathdly0)
{
    ASSERT_ERR((((uint32_t)txpathdly0 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM0_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM0_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)txpathdly0 << 0));
}

/**
 * @brief RADIOTXRXTIM1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:16            RFRXTMDA1   0x0
 *  14:08           RXPATHDLY1   0x0
 *  06:00           TXPATHDLY1   0x0
 * </pre>
 */
#define BLE_RADIOTXRXTIM1_ADDR   0x40100094
#define BLE_RADIOTXRXTIM1_OFFSET 0x00000094
#define BLE_RADIOTXRXTIM1_INDEX  0x00000025
#define BLE_RADIOTXRXTIM1_RESET  0x00000000

__INLINE uint32_t ble_radiotxrxtim1_get(void)
{
    return REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR);
}

__INLINE void ble_radiotxrxtim1_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOTXRXTIM1_ADDR, value);
}

// field definitions
#define BLE_RFRXTMDA1_MASK    ((uint32_t)0x007F0000)
#define BLE_RFRXTMDA1_LSB     16
#define BLE_RFRXTMDA1_WIDTH   ((uint32_t)0x00000007)
#define BLE_RXPATHDLY1_MASK   ((uint32_t)0x00007F00)
#define BLE_RXPATHDLY1_LSB    8
#define BLE_RXPATHDLY1_WIDTH  ((uint32_t)0x00000007)
#define BLE_TXPATHDLY1_MASK   ((uint32_t)0x0000007F)
#define BLE_TXPATHDLY1_LSB    0
#define BLE_TXPATHDLY1_WIDTH  ((uint32_t)0x00000007)

#define BLE_RFRXTMDA1_RST     0x0
#define BLE_RXPATHDLY1_RST    0x0
#define BLE_TXPATHDLY1_RST    0x0

__INLINE void ble_radiotxrxtim1_pack(uint8_t rfrxtmda1, uint8_t rxpathdly1, uint8_t txpathdly1)
{
    ASSERT_ERR((((uint32_t)rfrxtmda1 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxpathdly1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)txpathdly1 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM1_ADDR, ((uint32_t)rfrxtmda1 << 16) | ((uint32_t)rxpathdly1 << 8) | ((uint32_t)txpathdly1 << 0));
}

__INLINE void ble_radiotxrxtim1_unpack(uint8_t *rfrxtmda1, uint8_t *rxpathdly1, uint8_t *txpathdly1)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR);

    *rfrxtmda1 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *rxpathdly1 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *txpathdly1 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t ble_radiotxrxtim1_rfrxtmda1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void ble_radiotxrxtim1_rfrxtmda1_setf(uint8_t rfrxtmda1)
{
    ASSERT_ERR((((uint32_t)rfrxtmda1 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM1_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)rfrxtmda1 << 16));
}

__INLINE uint8_t ble_radiotxrxtim1_rxpathdly1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_radiotxrxtim1_rxpathdly1_setf(uint8_t rxpathdly1)
{
    ASSERT_ERR((((uint32_t)rxpathdly1 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM1_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)rxpathdly1 << 8));
}

__INLINE uint8_t ble_radiotxrxtim1_txpathdly1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void ble_radiotxrxtim1_txpathdly1_setf(uint8_t txpathdly1)
{
    ASSERT_ERR((((uint32_t)txpathdly1 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM1_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM1_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)txpathdly1 << 0));
}

/**
 * @brief RADIOTXRXTIM2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24      RXFLUSHPATHDLY2   0x0
 *  23:16            RFRXTMDA2   0x0
 *  15:08           RXPATHDLY2   0x0
 *  06:00           TXPATHDLY2   0x0
 * </pre>
 */
#define BLE_RADIOTXRXTIM2_ADDR   0x40100098
#define BLE_RADIOTXRXTIM2_OFFSET 0x00000098
#define BLE_RADIOTXRXTIM2_INDEX  0x00000026
#define BLE_RADIOTXRXTIM2_RESET  0x00000000

__INLINE uint32_t ble_radiotxrxtim2_get(void)
{
    return REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR);
}

__INLINE void ble_radiotxrxtim2_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOTXRXTIM2_ADDR, value);
}

// field definitions
#define BLE_RXFLUSHPATHDLY2_MASK   ((uint32_t)0xFF000000)
#define BLE_RXFLUSHPATHDLY2_LSB    24
#define BLE_RXFLUSHPATHDLY2_WIDTH  ((uint32_t)0x00000008)
#define BLE_RFRXTMDA2_MASK         ((uint32_t)0x00FF0000)
#define BLE_RFRXTMDA2_LSB          16
#define BLE_RFRXTMDA2_WIDTH        ((uint32_t)0x00000008)
#define BLE_RXPATHDLY2_MASK        ((uint32_t)0x0000FF00)
#define BLE_RXPATHDLY2_LSB         8
#define BLE_RXPATHDLY2_WIDTH       ((uint32_t)0x00000008)
#define BLE_TXPATHDLY2_MASK        ((uint32_t)0x0000007F)
#define BLE_TXPATHDLY2_LSB         0
#define BLE_TXPATHDLY2_WIDTH       ((uint32_t)0x00000007)

#define BLE_RXFLUSHPATHDLY2_RST    0x0
#define BLE_RFRXTMDA2_RST          0x0
#define BLE_RXPATHDLY2_RST         0x0
#define BLE_TXPATHDLY2_RST         0x0

__INLINE void ble_radiotxrxtim2_pack(uint8_t rxflushpathdly2, uint8_t rfrxtmda2, uint8_t rxpathdly2, uint8_t txpathdly2)
{
    ASSERT_ERR((((uint32_t)rxflushpathdly2 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rfrxtmda2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)rxpathdly2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)txpathdly2 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM2_ADDR, ((uint32_t)rxflushpathdly2 << 24) | ((uint32_t)rfrxtmda2 << 16) | ((uint32_t)rxpathdly2 << 8) | ((uint32_t)txpathdly2 << 0));
}

__INLINE void ble_radiotxrxtim2_unpack(uint8_t *rxflushpathdly2, uint8_t *rfrxtmda2, uint8_t *rxpathdly2, uint8_t *txpathdly2)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR);

    *rxflushpathdly2 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rfrxtmda2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *rxpathdly2 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *txpathdly2 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t ble_radiotxrxtim2_rxflushpathdly2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void ble_radiotxrxtim2_rxflushpathdly2_setf(uint8_t rxflushpathdly2)
{
    ASSERT_ERR((((uint32_t)rxflushpathdly2 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM2_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)rxflushpathdly2 << 24));
}

__INLINE uint8_t ble_radiotxrxtim2_rfrxtmda2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_radiotxrxtim2_rfrxtmda2_setf(uint8_t rfrxtmda2)
{
    ASSERT_ERR((((uint32_t)rfrxtmda2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM2_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rfrxtmda2 << 16));
}

__INLINE uint8_t ble_radiotxrxtim2_rxpathdly2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void ble_radiotxrxtim2_rxpathdly2_setf(uint8_t rxpathdly2)
{
    ASSERT_ERR((((uint32_t)rxpathdly2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM2_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)rxpathdly2 << 8));
}

__INLINE uint8_t ble_radiotxrxtim2_txpathdly2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void ble_radiotxrxtim2_txpathdly2_setf(uint8_t txpathdly2)
{
    ASSERT_ERR((((uint32_t)txpathdly2 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM2_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM2_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)txpathdly2 << 0));
}

/**
 * @brief RADIOTXRXTIM3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24      RXFLUSHPATHDLY3   0x0
 *  22:16            RFRXTMDA3   0x0
 *  06:00           TXPATHDLY3   0x0
 * </pre>
 */
#define BLE_RADIOTXRXTIM3_ADDR   0x4010009C
#define BLE_RADIOTXRXTIM3_OFFSET 0x0000009C
#define BLE_RADIOTXRXTIM3_INDEX  0x00000027
#define BLE_RADIOTXRXTIM3_RESET  0x00000000

__INLINE uint32_t ble_radiotxrxtim3_get(void)
{
    return REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR);
}

__INLINE void ble_radiotxrxtim3_set(uint32_t value)
{
    REG_BLE_WR(BLE_RADIOTXRXTIM3_ADDR, value);
}

// field definitions
#define BLE_RXFLUSHPATHDLY3_MASK   ((uint32_t)0xFF000000)
#define BLE_RXFLUSHPATHDLY3_LSB    24
#define BLE_RXFLUSHPATHDLY3_WIDTH  ((uint32_t)0x00000008)
#define BLE_RFRXTMDA3_MASK         ((uint32_t)0x007F0000)
#define BLE_RFRXTMDA3_LSB          16
#define BLE_RFRXTMDA3_WIDTH        ((uint32_t)0x00000007)
#define BLE_TXPATHDLY3_MASK        ((uint32_t)0x0000007F)
#define BLE_TXPATHDLY3_LSB         0
#define BLE_TXPATHDLY3_WIDTH       ((uint32_t)0x00000007)

#define BLE_RXFLUSHPATHDLY3_RST    0x0
#define BLE_RFRXTMDA3_RST          0x0
#define BLE_TXPATHDLY3_RST         0x0

__INLINE void ble_radiotxrxtim3_pack(uint8_t rxflushpathdly3, uint8_t rfrxtmda3, uint8_t txpathdly3)
{
    ASSERT_ERR((((uint32_t)rxflushpathdly3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rfrxtmda3 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)txpathdly3 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM3_ADDR, ((uint32_t)rxflushpathdly3 << 24) | ((uint32_t)rfrxtmda3 << 16) | ((uint32_t)txpathdly3 << 0));
}

__INLINE void ble_radiotxrxtim3_unpack(uint8_t *rxflushpathdly3, uint8_t *rfrxtmda3, uint8_t *txpathdly3)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR);

    *rxflushpathdly3 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rfrxtmda3 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *txpathdly3 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t ble_radiotxrxtim3_rxflushpathdly3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void ble_radiotxrxtim3_rxflushpathdly3_setf(uint8_t rxflushpathdly3)
{
    ASSERT_ERR((((uint32_t)rxflushpathdly3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM3_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)rxflushpathdly3 << 24));
}

__INLINE uint8_t ble_radiotxrxtim3_rfrxtmda3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void ble_radiotxrxtim3_rfrxtmda3_setf(uint8_t rfrxtmda3)
{
    ASSERT_ERR((((uint32_t)rfrxtmda3 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM3_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)rfrxtmda3 << 16));
}

__INLINE uint8_t ble_radiotxrxtim3_txpathdly3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void ble_radiotxrxtim3_txpathdly3_setf(uint8_t txpathdly3)
{
    ASSERT_ERR((((uint32_t)txpathdly3 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_RADIOTXRXTIM3_ADDR, (REG_BLE_RD(BLE_RADIOTXRXTIM3_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)txpathdly3 << 0));
}

/**
 * @brief SPIPTRCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16             TXOFFPTR   0x0
 *  15:00              TXONPTR   0x0
 * </pre>
 */
#define BLE_SPIPTRCNTL0_ADDR   0x401000A0
#define BLE_SPIPTRCNTL0_OFFSET 0x000000A0
#define BLE_SPIPTRCNTL0_INDEX  0x00000028
#define BLE_SPIPTRCNTL0_RESET  0x00000000

__INLINE uint32_t ble_spiptrcntl0_get(void)
{
    return REG_BLE_RD(BLE_SPIPTRCNTL0_ADDR);
}

__INLINE void ble_spiptrcntl0_set(uint32_t value)
{
    REG_BLE_WR(BLE_SPIPTRCNTL0_ADDR, value);
}

// field definitions
#define BLE_TXOFFPTR_MASK   ((uint32_t)0xFFFF0000)
#define BLE_TXOFFPTR_LSB    16
#define BLE_TXOFFPTR_WIDTH  ((uint32_t)0x00000010)
#define BLE_TXONPTR_MASK    ((uint32_t)0x0000FFFF)
#define BLE_TXONPTR_LSB     0
#define BLE_TXONPTR_WIDTH   ((uint32_t)0x00000010)

#define BLE_TXOFFPTR_RST    0x0
#define BLE_TXONPTR_RST     0x0

__INLINE void ble_spiptrcntl0_pack(uint16_t txoffptr, uint16_t txonptr)
{
    ASSERT_ERR((((uint32_t)txoffptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)txonptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL0_ADDR, ((uint32_t)txoffptr << 16) | ((uint32_t)txonptr << 0));
}

__INLINE void ble_spiptrcntl0_unpack(uint16_t *txoffptr, uint16_t *txonptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL0_ADDR);

    *txoffptr = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *txonptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_spiptrcntl0_txoffptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL0_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_spiptrcntl0_txoffptr_setf(uint16_t txoffptr)
{
    ASSERT_ERR((((uint32_t)txoffptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL0_ADDR, (REG_BLE_RD(BLE_SPIPTRCNTL0_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)txoffptr << 16));
}

__INLINE uint16_t ble_spiptrcntl0_txonptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_spiptrcntl0_txonptr_setf(uint16_t txonptr)
{
    ASSERT_ERR((((uint32_t)txonptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL0_ADDR, (REG_BLE_RD(BLE_SPIPTRCNTL0_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)txonptr << 0));
}

/**
 * @brief SPIPTRCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16             RXOFFPTR   0x0
 *  15:00              RXONPTR   0x0
 * </pre>
 */
#define BLE_SPIPTRCNTL1_ADDR   0x401000A4
#define BLE_SPIPTRCNTL1_OFFSET 0x000000A4
#define BLE_SPIPTRCNTL1_INDEX  0x00000029
#define BLE_SPIPTRCNTL1_RESET  0x00000000

__INLINE uint32_t ble_spiptrcntl1_get(void)
{
    return REG_BLE_RD(BLE_SPIPTRCNTL1_ADDR);
}

__INLINE void ble_spiptrcntl1_set(uint32_t value)
{
    REG_BLE_WR(BLE_SPIPTRCNTL1_ADDR, value);
}

// field definitions
#define BLE_RXOFFPTR_MASK   ((uint32_t)0xFFFF0000)
#define BLE_RXOFFPTR_LSB    16
#define BLE_RXOFFPTR_WIDTH  ((uint32_t)0x00000010)
#define BLE_RXONPTR_MASK    ((uint32_t)0x0000FFFF)
#define BLE_RXONPTR_LSB     0
#define BLE_RXONPTR_WIDTH   ((uint32_t)0x00000010)

#define BLE_RXOFFPTR_RST    0x0
#define BLE_RXONPTR_RST     0x0

__INLINE void ble_spiptrcntl1_pack(uint16_t rxoffptr, uint16_t rxonptr)
{
    ASSERT_ERR((((uint32_t)rxoffptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)rxonptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL1_ADDR, ((uint32_t)rxoffptr << 16) | ((uint32_t)rxonptr << 0));
}

__INLINE void ble_spiptrcntl1_unpack(uint16_t *rxoffptr, uint16_t *rxonptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL1_ADDR);

    *rxoffptr = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *rxonptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_spiptrcntl1_rxoffptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL1_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_spiptrcntl1_rxoffptr_setf(uint16_t rxoffptr)
{
    ASSERT_ERR((((uint32_t)rxoffptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL1_ADDR, (REG_BLE_RD(BLE_SPIPTRCNTL1_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)rxoffptr << 16));
}

__INLINE uint16_t ble_spiptrcntl1_rxonptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_spiptrcntl1_rxonptr_setf(uint16_t rxonptr)
{
    ASSERT_ERR((((uint32_t)rxonptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL1_ADDR, (REG_BLE_RD(BLE_SPIPTRCNTL1_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)rxonptr << 0));
}

/**
 * @brief SPIPTRCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16          RXLENGTHPTR   0x0
 *  15:00              RSSIPTR   0x0
 * </pre>
 */
#define BLE_SPIPTRCNTL2_ADDR   0x401000A8
#define BLE_SPIPTRCNTL2_OFFSET 0x000000A8
#define BLE_SPIPTRCNTL2_INDEX  0x0000002A
#define BLE_SPIPTRCNTL2_RESET  0x00000000

__INLINE uint32_t ble_spiptrcntl2_get(void)
{
    return REG_BLE_RD(BLE_SPIPTRCNTL2_ADDR);
}

__INLINE void ble_spiptrcntl2_set(uint32_t value)
{
    REG_BLE_WR(BLE_SPIPTRCNTL2_ADDR, value);
}

// field definitions
#define BLE_RXLENGTHPTR_MASK   ((uint32_t)0xFFFF0000)
#define BLE_RXLENGTHPTR_LSB    16
#define BLE_RXLENGTHPTR_WIDTH  ((uint32_t)0x00000010)
#define BLE_RSSIPTR_MASK       ((uint32_t)0x0000FFFF)
#define BLE_RSSIPTR_LSB        0
#define BLE_RSSIPTR_WIDTH      ((uint32_t)0x00000010)

#define BLE_RXLENGTHPTR_RST    0x0
#define BLE_RSSIPTR_RST        0x0

__INLINE void ble_spiptrcntl2_pack(uint16_t rxlengthptr, uint16_t rssiptr)
{
    ASSERT_ERR((((uint32_t)rxlengthptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)rssiptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL2_ADDR, ((uint32_t)rxlengthptr << 16) | ((uint32_t)rssiptr << 0));
}

__INLINE void ble_spiptrcntl2_unpack(uint16_t *rxlengthptr, uint16_t *rssiptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL2_ADDR);

    *rxlengthptr = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *rssiptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_spiptrcntl2_rxlengthptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL2_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_spiptrcntl2_rxlengthptr_setf(uint16_t rxlengthptr)
{
    ASSERT_ERR((((uint32_t)rxlengthptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL2_ADDR, (REG_BLE_RD(BLE_SPIPTRCNTL2_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)rxlengthptr << 16));
}

__INLINE uint16_t ble_spiptrcntl2_rssiptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_spiptrcntl2_rssiptr_setf(uint16_t rssiptr)
{
    ASSERT_ERR((((uint32_t)rssiptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL2_ADDR, (REG_BLE_RD(BLE_SPIPTRCNTL2_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)rssiptr << 0));
}

/**
 * @brief SPIPTRCNTL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          RXPKTTYPPTR   0x0
 * </pre>
 */
#define BLE_SPIPTRCNTL3_ADDR   0x401000AC
#define BLE_SPIPTRCNTL3_OFFSET 0x000000AC
#define BLE_SPIPTRCNTL3_INDEX  0x0000002B
#define BLE_SPIPTRCNTL3_RESET  0x00000000

__INLINE uint32_t ble_spiptrcntl3_get(void)
{
    return REG_BLE_RD(BLE_SPIPTRCNTL3_ADDR);
}

__INLINE void ble_spiptrcntl3_set(uint32_t value)
{
    REG_BLE_WR(BLE_SPIPTRCNTL3_ADDR, value);
}

// field definitions
#define BLE_RXPKTTYPPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_RXPKTTYPPTR_LSB    0
#define BLE_RXPKTTYPPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_RXPKTTYPPTR_RST    0x0

__INLINE uint16_t ble_spiptrcntl3_rxpkttypptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SPIPTRCNTL3_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_spiptrcntl3_rxpkttypptr_setf(uint16_t rxpkttypptr)
{
    ASSERT_ERR((((uint32_t)rxpkttypptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_SPIPTRCNTL3_ADDR, (uint32_t)rxpkttypptr << 0);
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
#define BLE_AESCNTL_ADDR   0x401000B0
#define BLE_AESCNTL_OFFSET 0x000000B0
#define BLE_AESCNTL_INDEX  0x0000002C
#define BLE_AESCNTL_RESET  0x00000000

__INLINE uint32_t ble_aescntl_get(void)
{
    return REG_BLE_RD(BLE_AESCNTL_ADDR);
}

__INLINE void ble_aescntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_AESCNTL_ADDR, value);
}

// field definitions
#define BLE_AES_MODE_BIT     ((uint32_t)0x00000002)
#define BLE_AES_MODE_POS     1
#define BLE_AES_START_BIT    ((uint32_t)0x00000001)
#define BLE_AES_START_POS    0

#define BLE_AES_MODE_RST     0x0
#define BLE_AES_START_RST    0x0

__INLINE void ble_aescntl_pack(uint8_t aesmode, uint8_t aesstart)
{
    ASSERT_ERR((((uint32_t)aesmode << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)aesstart << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_AESCNTL_ADDR, ((uint32_t)aesmode << 1) | ((uint32_t)aesstart << 0));
}

__INLINE void ble_aescntl_unpack(uint8_t *aesmode, uint8_t *aesstart)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESCNTL_ADDR);

    *aesmode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *aesstart = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_aescntl_aes_mode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ble_aescntl_aes_mode_setf(uint8_t aesmode)
{
    ASSERT_ERR((((uint32_t)aesmode << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BLE_WR(BLE_AESCNTL_ADDR, (REG_BLE_RD(BLE_AESCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)aesmode << 1));
}

__INLINE uint8_t ble_aescntl_aes_start_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_aescntl_aes_start_setf(uint8_t aesstart)
{
    ASSERT_ERR((((uint32_t)aesstart << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_AESCNTL_ADDR, (REG_BLE_RD(BLE_AESCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)aesstart << 0));
}

/**
 * @brief AESKEY31_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           AESKEY31_0   0x0
 * </pre>
 */
#define BLE_AESKEY31_0_ADDR   0x401000B4
#define BLE_AESKEY31_0_OFFSET 0x000000B4
#define BLE_AESKEY31_0_INDEX  0x0000002D
#define BLE_AESKEY31_0_RESET  0x00000000

__INLINE uint32_t ble_aeskey31_0_get(void)
{
    return REG_BLE_RD(BLE_AESKEY31_0_ADDR);
}

__INLINE void ble_aeskey31_0_set(uint32_t value)
{
    REG_BLE_WR(BLE_AESKEY31_0_ADDR, value);
}

// field definitions
#define BLE_AESKEY31_0_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_AESKEY31_0_LSB    0
#define BLE_AESKEY31_0_WIDTH  ((uint32_t)0x00000020)

#define BLE_AESKEY31_0_RST    0x0

__INLINE uint32_t ble_aeskey31_0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESKEY31_0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_aeskey31_0_setf(uint32_t aeskey310)
{
    ASSERT_ERR((((uint32_t)aeskey310 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_AESKEY31_0_ADDR, (uint32_t)aeskey310 << 0);
}

/**
 * @brief AESKEY63_32 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          AESKEY63_32   0x0
 * </pre>
 */
#define BLE_AESKEY63_32_ADDR   0x401000B8
#define BLE_AESKEY63_32_OFFSET 0x000000B8
#define BLE_AESKEY63_32_INDEX  0x0000002E
#define BLE_AESKEY63_32_RESET  0x00000000

__INLINE uint32_t ble_aeskey63_32_get(void)
{
    return REG_BLE_RD(BLE_AESKEY63_32_ADDR);
}

__INLINE void ble_aeskey63_32_set(uint32_t value)
{
    REG_BLE_WR(BLE_AESKEY63_32_ADDR, value);
}

// field definitions
#define BLE_AESKEY63_32_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_AESKEY63_32_LSB    0
#define BLE_AESKEY63_32_WIDTH  ((uint32_t)0x00000020)

#define BLE_AESKEY63_32_RST    0x0

__INLINE uint32_t ble_aeskey63_32_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESKEY63_32_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_aeskey63_32_setf(uint32_t aeskey6332)
{
    ASSERT_ERR((((uint32_t)aeskey6332 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_AESKEY63_32_ADDR, (uint32_t)aeskey6332 << 0);
}

/**
 * @brief AESKEY95_64 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          AESKEY95_64   0x0
 * </pre>
 */
#define BLE_AESKEY95_64_ADDR   0x401000BC
#define BLE_AESKEY95_64_OFFSET 0x000000BC
#define BLE_AESKEY95_64_INDEX  0x0000002F
#define BLE_AESKEY95_64_RESET  0x00000000

__INLINE uint32_t ble_aeskey95_64_get(void)
{
    return REG_BLE_RD(BLE_AESKEY95_64_ADDR);
}

__INLINE void ble_aeskey95_64_set(uint32_t value)
{
    REG_BLE_WR(BLE_AESKEY95_64_ADDR, value);
}

// field definitions
#define BLE_AESKEY95_64_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_AESKEY95_64_LSB    0
#define BLE_AESKEY95_64_WIDTH  ((uint32_t)0x00000020)

#define BLE_AESKEY95_64_RST    0x0

__INLINE uint32_t ble_aeskey95_64_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESKEY95_64_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_aeskey95_64_setf(uint32_t aeskey9564)
{
    ASSERT_ERR((((uint32_t)aeskey9564 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_AESKEY95_64_ADDR, (uint32_t)aeskey9564 << 0);
}

/**
 * @brief AESKEY127_96 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         AESKEY127_96   0x0
 * </pre>
 */
#define BLE_AESKEY127_96_ADDR   0x401000C0
#define BLE_AESKEY127_96_OFFSET 0x000000C0
#define BLE_AESKEY127_96_INDEX  0x00000030
#define BLE_AESKEY127_96_RESET  0x00000000

__INLINE uint32_t ble_aeskey127_96_get(void)
{
    return REG_BLE_RD(BLE_AESKEY127_96_ADDR);
}

__INLINE void ble_aeskey127_96_set(uint32_t value)
{
    REG_BLE_WR(BLE_AESKEY127_96_ADDR, value);
}

// field definitions
#define BLE_AESKEY127_96_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_AESKEY127_96_LSB    0
#define BLE_AESKEY127_96_WIDTH  ((uint32_t)0x00000020)

#define BLE_AESKEY127_96_RST    0x0

__INLINE uint32_t ble_aeskey127_96_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESKEY127_96_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_aeskey127_96_setf(uint32_t aeskey12796)
{
    ASSERT_ERR((((uint32_t)aeskey12796 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_AESKEY127_96_ADDR, (uint32_t)aeskey12796 << 0);
}

/**
 * @brief AESPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               AESPTR   0x0
 * </pre>
 */
#define BLE_AESPTR_ADDR   0x401000C4
#define BLE_AESPTR_OFFSET 0x000000C4
#define BLE_AESPTR_INDEX  0x00000031
#define BLE_AESPTR_RESET  0x00000000

__INLINE uint32_t ble_aesptr_get(void)
{
    return REG_BLE_RD(BLE_AESPTR_ADDR);
}

__INLINE void ble_aesptr_set(uint32_t value)
{
    REG_BLE_WR(BLE_AESPTR_ADDR, value);
}

// field definitions
#define BLE_AESPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_AESPTR_LSB    0
#define BLE_AESPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_AESPTR_RST    0x0

__INLINE uint16_t ble_aesptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_AESPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_aesptr_setf(uint16_t aesptr)
{
    ASSERT_ERR((((uint32_t)aesptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_AESPTR_ADDR, (uint32_t)aesptr << 0);
}

/**
 * @brief TXMICVAL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             TXMICVAL   0x0
 * </pre>
 */
#define BLE_TXMICVAL_ADDR   0x401000C8
#define BLE_TXMICVAL_OFFSET 0x000000C8
#define BLE_TXMICVAL_INDEX  0x00000032
#define BLE_TXMICVAL_RESET  0x00000000

__INLINE uint32_t ble_txmicval_get(void)
{
    return REG_BLE_RD(BLE_TXMICVAL_ADDR);
}

// field definitions
#define BLE_TXMICVAL_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_TXMICVAL_LSB    0
#define BLE_TXMICVAL_WIDTH  ((uint32_t)0x00000020)

#define BLE_TXMICVAL_RST    0x0

__INLINE uint32_t ble_txmicval_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_TXMICVAL_ADDR);
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
#define BLE_RXMICVAL_ADDR   0x401000CC
#define BLE_RXMICVAL_OFFSET 0x000000CC
#define BLE_RXMICVAL_INDEX  0x00000033
#define BLE_RXMICVAL_RESET  0x00000000

__INLINE uint32_t ble_rxmicval_get(void)
{
    return REG_BLE_RD(BLE_RXMICVAL_ADDR);
}

// field definitions
#define BLE_RXMICVAL_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_RXMICVAL_LSB    0
#define BLE_RXMICVAL_WIDTH  ((uint32_t)0x00000020)

#define BLE_RXMICVAL_RST    0x0

__INLINE uint32_t ble_rxmicval_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RXMICVAL_ADDR);
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
 *  25:24        PERCOUNT_MODE   0x0
 *     15           INFINITETX   0
 *     14          TXLENGTHSRC   0
 *     13             PRBSTYPE   0
 *     12             TXPLDSRC   0
 *     11           TXPKTCNTEN   0
 *  08:00             TXLENGTH   0x0
 * </pre>
 */
#define BLE_RFTESTCNTL_ADDR   0x401000D0
#define BLE_RFTESTCNTL_OFFSET 0x000000D0
#define BLE_RFTESTCNTL_INDEX  0x00000034
#define BLE_RFTESTCNTL_RESET  0x00000000

__INLINE uint32_t ble_rftestcntl_get(void)
{
    return REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
}

__INLINE void ble_rftestcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, value);
}

// field definitions
#define BLE_INFINITERX_BIT       ((uint32_t)0x80000000)
#define BLE_INFINITERX_POS       31
#define BLE_RXPKTCNTEN_BIT       ((uint32_t)0x08000000)
#define BLE_RXPKTCNTEN_POS       27
#define BLE_PERCOUNT_MODE_MASK   ((uint32_t)0x03000000)
#define BLE_PERCOUNT_MODE_LSB    24
#define BLE_PERCOUNT_MODE_WIDTH  ((uint32_t)0x00000002)
#define BLE_INFINITETX_BIT       ((uint32_t)0x00008000)
#define BLE_INFINITETX_POS       15
#define BLE_TXLENGTHSRC_BIT      ((uint32_t)0x00004000)
#define BLE_TXLENGTHSRC_POS      14
#define BLE_PRBSTYPE_BIT         ((uint32_t)0x00002000)
#define BLE_PRBSTYPE_POS         13
#define BLE_TXPLDSRC_BIT         ((uint32_t)0x00001000)
#define BLE_TXPLDSRC_POS         12
#define BLE_TXPKTCNTEN_BIT       ((uint32_t)0x00000800)
#define BLE_TXPKTCNTEN_POS       11
#define BLE_TXLENGTH_MASK        ((uint32_t)0x000001FF)
#define BLE_TXLENGTH_LSB         0
#define BLE_TXLENGTH_WIDTH       ((uint32_t)0x00000009)

#define BLE_INFINITERX_RST       0x0
#define BLE_RXPKTCNTEN_RST       0x0
#define BLE_PERCOUNT_MODE_RST    0x0
#define BLE_INFINITETX_RST       0x0
#define BLE_TXLENGTHSRC_RST      0x0
#define BLE_PRBSTYPE_RST         0x0
#define BLE_TXPLDSRC_RST         0x0
#define BLE_TXPKTCNTEN_RST       0x0
#define BLE_TXLENGTH_RST         0x0

__INLINE void ble_rftestcntl_pack(uint8_t infiniterx, uint8_t rxpktcnten, uint8_t percountmode, uint8_t infinitetx, uint8_t txlengthsrc, uint8_t prbstype, uint8_t txpldsrc, uint8_t txpktcnten, uint16_t txlength)
{
    ASSERT_ERR((((uint32_t)infiniterx << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)rxpktcnten << 27) & ~((uint32_t)0x08000000)) == 0);
    ASSERT_ERR((((uint32_t)percountmode << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)infinitetx << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)txlengthsrc << 14) & ~((uint32_t)0x00004000)) == 0);
    ASSERT_ERR((((uint32_t)prbstype << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)txpldsrc << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)txpktcnten << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)txlength << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, ((uint32_t)infiniterx << 31) | ((uint32_t)rxpktcnten << 27) | ((uint32_t)percountmode << 24) | ((uint32_t)infinitetx << 15) | ((uint32_t)txlengthsrc << 14) | ((uint32_t)prbstype << 13) | ((uint32_t)txpldsrc << 12) | ((uint32_t)txpktcnten << 11) | ((uint32_t)txlength << 0));
}

__INLINE void ble_rftestcntl_unpack(uint8_t *infiniterx, uint8_t *rxpktcnten, uint8_t *percountmode, uint8_t *infinitetx, uint8_t *txlengthsrc, uint8_t *prbstype, uint8_t *txpldsrc, uint8_t *txpktcnten, uint16_t *txlength)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);

    *infiniterx = (localVal & ((uint32_t)0x80000000)) >> 31;
    *rxpktcnten = (localVal & ((uint32_t)0x08000000)) >> 27;
    *percountmode = (localVal & ((uint32_t)0x03000000)) >> 24;
    *infinitetx = (localVal & ((uint32_t)0x00008000)) >> 15;
    *txlengthsrc = (localVal & ((uint32_t)0x00004000)) >> 14;
    *prbstype = (localVal & ((uint32_t)0x00002000)) >> 13;
    *txpldsrc = (localVal & ((uint32_t)0x00001000)) >> 12;
    *txpktcnten = (localVal & ((uint32_t)0x00000800)) >> 11;
    *txlength = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint8_t ble_rftestcntl_infiniterx_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_rftestcntl_infiniterx_setf(uint8_t infiniterx)
{
    ASSERT_ERR((((uint32_t)infiniterx << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)infiniterx << 31));
}

__INLINE uint8_t ble_rftestcntl_rxpktcnten_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void ble_rftestcntl_rxpktcnten_setf(uint8_t rxpktcnten)
{
    ASSERT_ERR((((uint32_t)rxpktcnten << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)rxpktcnten << 27));
}

__INLINE uint8_t ble_rftestcntl_percount_mode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void ble_rftestcntl_percount_mode_setf(uint8_t percountmode)
{
    ASSERT_ERR((((uint32_t)percountmode << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)percountmode << 24));
}

__INLINE uint8_t ble_rftestcntl_infinitetx_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ble_rftestcntl_infinitetx_setf(uint8_t infinitetx)
{
    ASSERT_ERR((((uint32_t)infinitetx << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)infinitetx << 15));
}

__INLINE uint8_t ble_rftestcntl_txlengthsrc_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ble_rftestcntl_txlengthsrc_setf(uint8_t txlengthsrc)
{
    ASSERT_ERR((((uint32_t)txlengthsrc << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)txlengthsrc << 14));
}

__INLINE uint8_t ble_rftestcntl_prbstype_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ble_rftestcntl_prbstype_setf(uint8_t prbstype)
{
    ASSERT_ERR((((uint32_t)prbstype << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)prbstype << 13));
}

__INLINE uint8_t ble_rftestcntl_txpldsrc_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ble_rftestcntl_txpldsrc_setf(uint8_t txpldsrc)
{
    ASSERT_ERR((((uint32_t)txpldsrc << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)txpldsrc << 12));
}

__INLINE uint8_t ble_rftestcntl_txpktcnten_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ble_rftestcntl_txpktcnten_setf(uint8_t txpktcnten)
{
    ASSERT_ERR((((uint32_t)txpktcnten << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)txpktcnten << 11));
}

__INLINE uint16_t ble_rftestcntl_txlength_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void ble_rftestcntl_txlength_setf(uint16_t txlength)
{
    ASSERT_ERR((((uint32_t)txlength << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BLE_WR(BLE_RFTESTCNTL_ADDR, (REG_BLE_RD(BLE_RFTESTCNTL_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)txlength << 0));
}

/**
 * @brief RFTESTTXSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             TXPKTCNT   0x0
 * </pre>
 */
#define BLE_RFTESTTXSTAT_ADDR   0x401000D4
#define BLE_RFTESTTXSTAT_OFFSET 0x000000D4
#define BLE_RFTESTTXSTAT_INDEX  0x00000035
#define BLE_RFTESTTXSTAT_RESET  0x00000000

__INLINE uint32_t ble_rftesttxstat_get(void)
{
    return REG_BLE_RD(BLE_RFTESTTXSTAT_ADDR);
}

// field definitions
#define BLE_TXPKTCNT_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_TXPKTCNT_LSB    0
#define BLE_TXPKTCNT_WIDTH  ((uint32_t)0x00000020)

#define BLE_TXPKTCNT_RST    0x0

__INLINE uint32_t ble_rftesttxstat_txpktcnt_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTTXSTAT_ADDR);
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
#define BLE_RFTESTRXSTAT_ADDR   0x401000D8
#define BLE_RFTESTRXSTAT_OFFSET 0x000000D8
#define BLE_RFTESTRXSTAT_INDEX  0x00000036
#define BLE_RFTESTRXSTAT_RESET  0x00000000

__INLINE uint32_t ble_rftestrxstat_get(void)
{
    return REG_BLE_RD(BLE_RFTESTRXSTAT_ADDR);
}

// field definitions
#define BLE_RXPKTCNT_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_RXPKTCNT_LSB    0
#define BLE_RXPKTCNT_WIDTH  ((uint32_t)0x00000020)

#define BLE_RXPKTCNT_RST    0x0

__INLINE uint32_t ble_rftestrxstat_rxpktcnt_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RFTESTRXSTAT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief TIMGENCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:16   PREFETCHABORT_TIME   0x1FE
 *  08:00        PREFETCH_TIME   0x96
 * </pre>
 */
#define BLE_TIMGENCNTL_ADDR   0x401000E0
#define BLE_TIMGENCNTL_OFFSET 0x000000E0
#define BLE_TIMGENCNTL_INDEX  0x00000038
#define BLE_TIMGENCNTL_RESET  0x01FE0096

__INLINE uint32_t ble_timgencntl_get(void)
{
    return REG_BLE_RD(BLE_TIMGENCNTL_ADDR);
}

__INLINE void ble_timgencntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_TIMGENCNTL_ADDR, value);
}

// field definitions
#define BLE_PREFETCHABORT_TIME_MASK   ((uint32_t)0x03FF0000)
#define BLE_PREFETCHABORT_TIME_LSB    16
#define BLE_PREFETCHABORT_TIME_WIDTH  ((uint32_t)0x0000000A)
#define BLE_PREFETCH_TIME_MASK        ((uint32_t)0x000001FF)
#define BLE_PREFETCH_TIME_LSB         0
#define BLE_PREFETCH_TIME_WIDTH       ((uint32_t)0x00000009)

#define BLE_PREFETCHABORT_TIME_RST    0x1FE
#define BLE_PREFETCH_TIME_RST         0x96

__INLINE void ble_timgencntl_pack(uint16_t prefetchaborttime, uint16_t prefetchtime)
{
    ASSERT_ERR((((uint32_t)prefetchaborttime << 16) & ~((uint32_t)0x03FF0000)) == 0);
    ASSERT_ERR((((uint32_t)prefetchtime << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BLE_WR(BLE_TIMGENCNTL_ADDR, ((uint32_t)prefetchaborttime << 16) | ((uint32_t)prefetchtime << 0));
}

__INLINE void ble_timgencntl_unpack(uint16_t *prefetchaborttime, uint16_t *prefetchtime)
{
    uint32_t localVal = REG_BLE_RD(BLE_TIMGENCNTL_ADDR);

    *prefetchaborttime = (localVal & ((uint32_t)0x03FF0000)) >> 16;
    *prefetchtime = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t ble_timgencntl_prefetchabort_time_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_TIMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x03FF0000)) >> 16);
}

__INLINE void ble_timgencntl_prefetchabort_time_setf(uint16_t prefetchaborttime)
{
    ASSERT_ERR((((uint32_t)prefetchaborttime << 16) & ~((uint32_t)0x03FF0000)) == 0);
    REG_BLE_WR(BLE_TIMGENCNTL_ADDR, (REG_BLE_RD(BLE_TIMGENCNTL_ADDR) & ~((uint32_t)0x03FF0000)) | ((uint32_t)prefetchaborttime << 16));
}

__INLINE uint16_t ble_timgencntl_prefetch_time_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_TIMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void ble_timgencntl_prefetch_time_setf(uint16_t prefetchtime)
{
    ASSERT_ERR((((uint32_t)prefetchtime << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BLE_WR(BLE_TIMGENCNTL_ADDR, (REG_BLE_RD(BLE_TIMGENCNTL_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)prefetchtime << 0));
}

/**
 * @brief GROSSTIMTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:00          GROSSTARGET   0x0
 * </pre>
 */
#define BLE_GROSSTIMTGT_ADDR   0x401000E4
#define BLE_GROSSTIMTGT_OFFSET 0x000000E4
#define BLE_GROSSTIMTGT_INDEX  0x00000039
#define BLE_GROSSTIMTGT_RESET  0x00000000

__INLINE uint32_t ble_grosstimtgt_get(void)
{
    return REG_BLE_RD(BLE_GROSSTIMTGT_ADDR);
}

__INLINE void ble_grosstimtgt_set(uint32_t value)
{
    REG_BLE_WR(BLE_GROSSTIMTGT_ADDR, value);
}

// field definitions
#define BLE_GROSSTARGET_MASK   ((uint32_t)0x007FFFFF)
#define BLE_GROSSTARGET_LSB    0
#define BLE_GROSSTARGET_WIDTH  ((uint32_t)0x00000017)

#define BLE_GROSSTARGET_RST    0x0

__INLINE uint32_t ble_grosstimtgt_grosstarget_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_GROSSTIMTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x007FFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_grosstimtgt_grosstarget_setf(uint32_t grosstarget)
{
    ASSERT_ERR((((uint32_t)grosstarget << 0) & ~((uint32_t)0x007FFFFF)) == 0);
    REG_BLE_WR(BLE_GROSSTIMTGT_ADDR, (uint32_t)grosstarget << 0);
}

/**
 * @brief FINETIMTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00           FINETARGET   0x0
 * </pre>
 */
#define BLE_FINETIMTGT_ADDR   0x401000E8
#define BLE_FINETIMTGT_OFFSET 0x000000E8
#define BLE_FINETIMTGT_INDEX  0x0000003A
#define BLE_FINETIMTGT_RESET  0x00000000

__INLINE uint32_t ble_finetimtgt_get(void)
{
    return REG_BLE_RD(BLE_FINETIMTGT_ADDR);
}

__INLINE void ble_finetimtgt_set(uint32_t value)
{
    REG_BLE_WR(BLE_FINETIMTGT_ADDR, value);
}

// field definitions
#define BLE_FINETARGET_MASK   ((uint32_t)0x0FFFFFFF)
#define BLE_FINETARGET_LSB    0
#define BLE_FINETARGET_WIDTH  ((uint32_t)0x0000001C)

#define BLE_FINETARGET_RST    0x0

__INLINE uint32_t ble_finetimtgt_finetarget_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_FINETIMTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_finetimtgt_finetarget_setf(uint32_t finetarget)
{
    ASSERT_ERR((((uint32_t)finetarget << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BLE_WR(BLE_FINETIMTGT_ADDR, (uint32_t)finetarget << 0);
}

/**
 * @brief CLKNTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00              CLKNTGT   0x0
 * </pre>
 */
#define BLE_CLKNTGT_ADDR   0x401000EC
#define BLE_CLKNTGT_OFFSET 0x000000EC
#define BLE_CLKNTGT_INDEX  0x0000003B
#define BLE_CLKNTGT_RESET  0x00000000

__INLINE uint32_t ble_clkntgt_get(void)
{
    return REG_BLE_RD(BLE_CLKNTGT_ADDR);
}

__INLINE void ble_clkntgt_set(uint32_t value)
{
    REG_BLE_WR(BLE_CLKNTGT_ADDR, value);
}

// field definitions
#define BLE_CLKNTGT_MASK   ((uint32_t)0x0FFFFFFF)
#define BLE_CLKNTGT_LSB    0
#define BLE_CLKNTGT_WIDTH  ((uint32_t)0x0000001C)

#define BLE_CLKNTGT_RST    0x0

__INLINE uint32_t ble_clkntgt_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_CLKNTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_clkntgt_setf(uint32_t clkntgt)
{
    ASSERT_ERR((((uint32_t)clkntgt << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_BLE_WR(BLE_CLKNTGT_ADDR, (uint32_t)clkntgt << 0);
}

/**
 * @brief HMICROSECTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00         HMICROSECTGT   0x0
 * </pre>
 */
#define BLE_HMICROSECTGT_ADDR   0x401000F0
#define BLE_HMICROSECTGT_OFFSET 0x000000F0
#define BLE_HMICROSECTGT_INDEX  0x0000003C
#define BLE_HMICROSECTGT_RESET  0x00000000

__INLINE uint32_t ble_hmicrosectgt_get(void)
{
    return REG_BLE_RD(BLE_HMICROSECTGT_ADDR);
}

__INLINE void ble_hmicrosectgt_set(uint32_t value)
{
    REG_BLE_WR(BLE_HMICROSECTGT_ADDR, value);
}

// field definitions
#define BLE_HMICROSECTGT_MASK   ((uint32_t)0x000003FF)
#define BLE_HMICROSECTGT_LSB    0
#define BLE_HMICROSECTGT_WIDTH  ((uint32_t)0x0000000A)

#define BLE_HMICROSECTGT_RST    0x0

__INLINE uint16_t ble_hmicrosectgt_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_HMICROSECTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_hmicrosectgt_setf(uint16_t hmicrosectgt)
{
    ASSERT_ERR((((uint32_t)hmicrosectgt << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_BLE_WR(BLE_HMICROSECTGT_ADDR, (uint32_t)hmicrosectgt << 0);
}

/**
 * @brief LESCHCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            START_EVT   0
 *  03:00            ENTRY_IDX   0x0
 * </pre>
 */
#define BLE_LESCHCNTL_ADDR   0x40100100
#define BLE_LESCHCNTL_OFFSET 0x00000100
#define BLE_LESCHCNTL_INDEX  0x00000040
#define BLE_LESCHCNTL_RESET  0x00000000

__INLINE uint32_t ble_leschcntl_get(void)
{
    return REG_BLE_RD(BLE_LESCHCNTL_ADDR);
}

__INLINE void ble_leschcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_LESCHCNTL_ADDR, value);
}

// field definitions
#define BLE_START_EVT_BIT    ((uint32_t)0x80000000)
#define BLE_START_EVT_POS    31
#define BLE_ENTRY_IDX_MASK   ((uint32_t)0x0000000F)
#define BLE_ENTRY_IDX_LSB    0
#define BLE_ENTRY_IDX_WIDTH  ((uint32_t)0x00000004)

#define BLE_START_EVT_RST    0x0
#define BLE_ENTRY_IDX_RST    0x0

__INLINE void ble_leschcntl_pack(uint8_t startevt, uint8_t entryidx)
{
    ASSERT_ERR((((uint32_t)startevt << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)entryidx << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_LESCHCNTL_ADDR, ((uint32_t)startevt << 31) | ((uint32_t)entryidx << 0));
}

__INLINE void ble_leschcntl_unpack(uint8_t *startevt, uint8_t *entryidx)
{
    uint32_t localVal = REG_BLE_RD(BLE_LESCHCNTL_ADDR);

    *startevt = (localVal & ((uint32_t)0x80000000)) >> 31;
    *entryidx = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ble_leschcntl_start_evt_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_LESCHCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_leschcntl_start_evt_setf(uint8_t startevt)
{
    ASSERT_ERR((((uint32_t)startevt << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_LESCHCNTL_ADDR, (REG_BLE_RD(BLE_LESCHCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)startevt << 31));
}

__INLINE uint8_t ble_leschcntl_entry_idx_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_LESCHCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ble_leschcntl_entry_idx_setf(uint8_t entryidx)
{
    ASSERT_ERR((((uint32_t)entryidx << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_LESCHCNTL_ADDR, (REG_BLE_RD(BLE_LESCHCNTL_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)entryidx << 0));
}

/**
 * @brief STARTEVTCLKNTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00       STARTEVTCLKNTS   0x0
 * </pre>
 */
#define BLE_STARTEVTCLKNTS_ADDR   0x40100104
#define BLE_STARTEVTCLKNTS_OFFSET 0x00000104
#define BLE_STARTEVTCLKNTS_INDEX  0x00000041
#define BLE_STARTEVTCLKNTS_RESET  0x00000000

__INLINE uint32_t ble_startevtclknts_get(void)
{
    return REG_BLE_RD(BLE_STARTEVTCLKNTS_ADDR);
}

// field definitions
#define BLE_STARTEVTCLKNTS_MASK   ((uint32_t)0x0FFFFFFF)
#define BLE_STARTEVTCLKNTS_LSB    0
#define BLE_STARTEVTCLKNTS_WIDTH  ((uint32_t)0x0000001C)

#define BLE_STARTEVTCLKNTS_RST    0x0

__INLINE uint32_t ble_startevtclknts_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_STARTEVTCLKNTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief STARTEVTFINECNTTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00    STARTEVTFINECNTTS   0x0
 * </pre>
 */
#define BLE_STARTEVTFINECNTTS_ADDR   0x40100108
#define BLE_STARTEVTFINECNTTS_OFFSET 0x00000108
#define BLE_STARTEVTFINECNTTS_INDEX  0x00000042
#define BLE_STARTEVTFINECNTTS_RESET  0x00000000

__INLINE uint32_t ble_startevtfinecntts_get(void)
{
    return REG_BLE_RD(BLE_STARTEVTFINECNTTS_ADDR);
}

// field definitions
#define BLE_STARTEVTFINECNTTS_MASK   ((uint32_t)0x000003FF)
#define BLE_STARTEVTFINECNTTS_LSB    0
#define BLE_STARTEVTFINECNTTS_WIDTH  ((uint32_t)0x0000000A)

#define BLE_STARTEVTFINECNTTS_RST    0x0

__INLINE uint16_t ble_startevtfinecntts_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_STARTEVTFINECNTTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ENDEVTCLKNTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00         ENDEVTCLKNTS   0x0
 * </pre>
 */
#define BLE_ENDEVTCLKNTS_ADDR   0x4010010C
#define BLE_ENDEVTCLKNTS_OFFSET 0x0000010C
#define BLE_ENDEVTCLKNTS_INDEX  0x00000043
#define BLE_ENDEVTCLKNTS_RESET  0x00000000

__INLINE uint32_t ble_endevtclknts_get(void)
{
    return REG_BLE_RD(BLE_ENDEVTCLKNTS_ADDR);
}

// field definitions
#define BLE_ENDEVTCLKNTS_MASK   ((uint32_t)0x0FFFFFFF)
#define BLE_ENDEVTCLKNTS_LSB    0
#define BLE_ENDEVTCLKNTS_WIDTH  ((uint32_t)0x0000001C)

#define BLE_ENDEVTCLKNTS_RST    0x0

__INLINE uint32_t ble_endevtclknts_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ENDEVTCLKNTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ENDEVTFINECNTTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00      ENDEVTFINECNTTS   0x0
 * </pre>
 */
#define BLE_ENDEVTFINECNTTS_ADDR   0x40100110
#define BLE_ENDEVTFINECNTTS_OFFSET 0x00000110
#define BLE_ENDEVTFINECNTTS_INDEX  0x00000044
#define BLE_ENDEVTFINECNTTS_RESET  0x00000000

__INLINE uint32_t ble_endevtfinecntts_get(void)
{
    return REG_BLE_RD(BLE_ENDEVTFINECNTTS_ADDR);
}

// field definitions
#define BLE_ENDEVTFINECNTTS_MASK   ((uint32_t)0x000003FF)
#define BLE_ENDEVTFINECNTTS_LSB    0
#define BLE_ENDEVTFINECNTTS_WIDTH  ((uint32_t)0x0000000A)

#define BLE_ENDEVTFINECNTTS_RST    0x0

__INLINE uint16_t ble_endevtfinecntts_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ENDEVTFINECNTTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief SKIPEVTCLKNTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00        SKIPEVTCLKNTS   0x0
 * </pre>
 */
#define BLE_SKIPEVTCLKNTS_ADDR   0x40100114
#define BLE_SKIPEVTCLKNTS_OFFSET 0x00000114
#define BLE_SKIPEVTCLKNTS_INDEX  0x00000045
#define BLE_SKIPEVTCLKNTS_RESET  0x00000000

__INLINE uint32_t ble_skipevtclknts_get(void)
{
    return REG_BLE_RD(BLE_SKIPEVTCLKNTS_ADDR);
}

// field definitions
#define BLE_SKIPEVTCLKNTS_MASK   ((uint32_t)0x0FFFFFFF)
#define BLE_SKIPEVTCLKNTS_LSB    0
#define BLE_SKIPEVTCLKNTS_WIDTH  ((uint32_t)0x0000001C)

#define BLE_SKIPEVTCLKNTS_RST    0x0

__INLINE uint32_t ble_skipevtclknts_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SKIPEVTCLKNTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief SKIPEVTFINECNTTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00     SKIPEVTFINECNTTS   0x0
 * </pre>
 */
#define BLE_SKIPEVTFINECNTTS_ADDR   0x40100118
#define BLE_SKIPEVTFINECNTTS_OFFSET 0x00000118
#define BLE_SKIPEVTFINECNTTS_INDEX  0x00000046
#define BLE_SKIPEVTFINECNTTS_RESET  0x00000000

__INLINE uint32_t ble_skipevtfinecntts_get(void)
{
    return REG_BLE_RD(BLE_SKIPEVTFINECNTTS_ADDR);
}

// field definitions
#define BLE_SKIPEVTFINECNTTS_MASK   ((uint32_t)0x000003FF)
#define BLE_SKIPEVTFINECNTTS_LSB    0
#define BLE_SKIPEVTFINECNTTS_WIDTH  ((uint32_t)0x0000000A)

#define BLE_SKIPEVTFINECNTTS_RST    0x0

__INLINE uint16_t ble_skipevtfinecntts_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SKIPEVTFINECNTTS_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ADVTIM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        TX_AUXPTR_THR   0x0
 *  23:16        RX_AUXPTR_THR   0x0
 *  13:00               ADVINT   0x0
 * </pre>
 */
#define BLE_ADVTIM_ADDR   0x40100120
#define BLE_ADVTIM_OFFSET 0x00000120
#define BLE_ADVTIM_INDEX  0x00000048
#define BLE_ADVTIM_RESET  0x00000000

__INLINE uint32_t ble_advtim_get(void)
{
    return REG_BLE_RD(BLE_ADVTIM_ADDR);
}

__INLINE void ble_advtim_set(uint32_t value)
{
    REG_BLE_WR(BLE_ADVTIM_ADDR, value);
}

// field definitions
#define BLE_TX_AUXPTR_THR_MASK   ((uint32_t)0xFF000000)
#define BLE_TX_AUXPTR_THR_LSB    24
#define BLE_TX_AUXPTR_THR_WIDTH  ((uint32_t)0x00000008)
#define BLE_RX_AUXPTR_THR_MASK   ((uint32_t)0x00FF0000)
#define BLE_RX_AUXPTR_THR_LSB    16
#define BLE_RX_AUXPTR_THR_WIDTH  ((uint32_t)0x00000008)
#define BLE_ADVINT_MASK          ((uint32_t)0x00003FFF)
#define BLE_ADVINT_LSB           0
#define BLE_ADVINT_WIDTH         ((uint32_t)0x0000000E)

#define BLE_TX_AUXPTR_THR_RST    0x0
#define BLE_RX_AUXPTR_THR_RST    0x0
#define BLE_ADVINT_RST           0x0

__INLINE void ble_advtim_pack(uint8_t txauxptrthr, uint8_t rxauxptrthr, uint16_t advint)
{
    ASSERT_ERR((((uint32_t)txauxptrthr << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rxauxptrthr << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)advint << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BLE_WR(BLE_ADVTIM_ADDR, ((uint32_t)txauxptrthr << 24) | ((uint32_t)rxauxptrthr << 16) | ((uint32_t)advint << 0));
}

__INLINE void ble_advtim_unpack(uint8_t *txauxptrthr, uint8_t *rxauxptrthr, uint16_t *advint)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADVTIM_ADDR);

    *txauxptrthr = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rxauxptrthr = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *advint = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t ble_advtim_tx_auxptr_thr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADVTIM_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void ble_advtim_tx_auxptr_thr_setf(uint8_t txauxptrthr)
{
    ASSERT_ERR((((uint32_t)txauxptrthr << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_BLE_WR(BLE_ADVTIM_ADDR, (REG_BLE_RD(BLE_ADVTIM_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)txauxptrthr << 24));
}

__INLINE uint8_t ble_advtim_rx_auxptr_thr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADVTIM_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_advtim_rx_auxptr_thr_setf(uint8_t rxauxptrthr)
{
    ASSERT_ERR((((uint32_t)rxauxptrthr << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_ADVTIM_ADDR, (REG_BLE_RD(BLE_ADVTIM_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxauxptrthr << 16));
}

__INLINE uint16_t ble_advtim_advint_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADVTIM_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void ble_advtim_advint_setf(uint16_t advint)
{
    ASSERT_ERR((((uint32_t)advint << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_BLE_WR(BLE_ADVTIM_ADDR, (REG_BLE_RD(BLE_ADVTIM_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)advint << 0));
}

/**
 * @brief ACTSCANCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  24:16              BACKOFF   0x1
 *  08:00           UPPERLIMIT   0x1
 * </pre>
 */
#define BLE_ACTSCANCNTL_ADDR   0x40100124
#define BLE_ACTSCANCNTL_OFFSET 0x00000124
#define BLE_ACTSCANCNTL_INDEX  0x00000049
#define BLE_ACTSCANCNTL_RESET  0x00010001

__INLINE uint32_t ble_actscancntl_get(void)
{
    return REG_BLE_RD(BLE_ACTSCANCNTL_ADDR);
}

__INLINE void ble_actscancntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_ACTSCANCNTL_ADDR, value);
}

// field definitions
#define BLE_BACKOFF_MASK      ((uint32_t)0x01FF0000)
#define BLE_BACKOFF_LSB       16
#define BLE_BACKOFF_WIDTH     ((uint32_t)0x00000009)
#define BLE_UPPERLIMIT_MASK   ((uint32_t)0x000001FF)
#define BLE_UPPERLIMIT_LSB    0
#define BLE_UPPERLIMIT_WIDTH  ((uint32_t)0x00000009)

#define BLE_BACKOFF_RST       0x1
#define BLE_UPPERLIMIT_RST    0x1

__INLINE void ble_actscancntl_pack(uint16_t backoff, uint16_t upperlimit)
{
    ASSERT_ERR((((uint32_t)backoff << 16) & ~((uint32_t)0x01FF0000)) == 0);
    ASSERT_ERR((((uint32_t)upperlimit << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BLE_WR(BLE_ACTSCANCNTL_ADDR, ((uint32_t)backoff << 16) | ((uint32_t)upperlimit << 0));
}

__INLINE void ble_actscancntl_unpack(uint16_t *backoff, uint16_t *upperlimit)
{
    uint32_t localVal = REG_BLE_RD(BLE_ACTSCANCNTL_ADDR);

    *backoff = (localVal & ((uint32_t)0x01FF0000)) >> 16;
    *upperlimit = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t ble_actscancntl_backoff_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ACTSCANCNTL_ADDR);
    return ((localVal & ((uint32_t)0x01FF0000)) >> 16);
}

__INLINE void ble_actscancntl_backoff_setf(uint16_t backoff)
{
    ASSERT_ERR((((uint32_t)backoff << 16) & ~((uint32_t)0x01FF0000)) == 0);
    REG_BLE_WR(BLE_ACTSCANCNTL_ADDR, (REG_BLE_RD(BLE_ACTSCANCNTL_ADDR) & ~((uint32_t)0x01FF0000)) | ((uint32_t)backoff << 16));
}

__INLINE uint16_t ble_actscancntl_upperlimit_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ACTSCANCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void ble_actscancntl_upperlimit_setf(uint16_t upperlimit)
{
    ASSERT_ERR((((uint32_t)upperlimit << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_BLE_WR(BLE_ACTSCANCNTL_ADDR, (REG_BLE_RD(BLE_ACTSCANCNTL_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)upperlimit << 0));
}

/**
 * @brief WLCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16              WLNBDEV   0x0
 *  15:00            WLBASEPTR   0x0
 * </pre>
 */
#define BLE_WLCNTL_ADDR   0x40100130
#define BLE_WLCNTL_OFFSET 0x00000130
#define BLE_WLCNTL_INDEX  0x0000004C
#define BLE_WLCNTL_RESET  0x00000000

__INLINE uint32_t ble_wlcntl_get(void)
{
    return REG_BLE_RD(BLE_WLCNTL_ADDR);
}

__INLINE void ble_wlcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_WLCNTL_ADDR, value);
}

// field definitions
#define BLE_WLNBDEV_MASK     ((uint32_t)0x00FF0000)
#define BLE_WLNBDEV_LSB      16
#define BLE_WLNBDEV_WIDTH    ((uint32_t)0x00000008)
#define BLE_WLBASEPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_WLBASEPTR_LSB    0
#define BLE_WLBASEPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_WLNBDEV_RST      0x0
#define BLE_WLBASEPTR_RST    0x0

__INLINE void ble_wlcntl_pack(uint8_t wlnbdev, uint16_t wlbaseptr)
{
    ASSERT_ERR((((uint32_t)wlnbdev << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)wlbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_WLCNTL_ADDR, ((uint32_t)wlnbdev << 16) | ((uint32_t)wlbaseptr << 0));
}

__INLINE void ble_wlcntl_unpack(uint8_t *wlnbdev, uint16_t *wlbaseptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_WLCNTL_ADDR);

    *wlnbdev = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *wlbaseptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t ble_wlcntl_wlnbdev_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_WLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_wlcntl_wlnbdev_setf(uint8_t wlnbdev)
{
    ASSERT_ERR((((uint32_t)wlnbdev << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_WLCNTL_ADDR, (REG_BLE_RD(BLE_WLCNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)wlnbdev << 16));
}

__INLINE uint16_t ble_wlcntl_wlbaseptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_WLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_wlcntl_wlbaseptr_setf(uint16_t wlbaseptr)
{
    ASSERT_ERR((((uint32_t)wlbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_WLCNTL_ADDR, (REG_BLE_RD(BLE_WLCNTL_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)wlbaseptr << 0));
}

/**
 * @brief WLCURRENTPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         WLCURRENTPTR   0x0
 * </pre>
 */
#define BLE_WLCURRENTPTR_ADDR   0x40100134
#define BLE_WLCURRENTPTR_OFFSET 0x00000134
#define BLE_WLCURRENTPTR_INDEX  0x0000004D
#define BLE_WLCURRENTPTR_RESET  0x00000000

__INLINE uint32_t ble_wlcurrentptr_get(void)
{
    return REG_BLE_RD(BLE_WLCURRENTPTR_ADDR);
}

__INLINE void ble_wlcurrentptr_set(uint32_t value)
{
    REG_BLE_WR(BLE_WLCURRENTPTR_ADDR, value);
}

// field definitions
#define BLE_WLCURRENTPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_WLCURRENTPTR_LSB    0
#define BLE_WLCURRENTPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_WLCURRENTPTR_RST    0x0

__INLINE uint16_t ble_wlcurrentptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_WLCURRENTPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_wlcurrentptr_setf(uint16_t wlcurrentptr)
{
    ASSERT_ERR((((uint32_t)wlcurrentptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_WLCURRENTPTR_ADDR, (uint32_t)wlcurrentptr << 0);
}

/**
 * @brief PERADVLCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16         PERADVLNBDEV   0x0
 *  15:00       PERADVLBASEPTR   0x0
 * </pre>
 */
#define BLE_PERADVLCNTL_ADDR   0x40100138
#define BLE_PERADVLCNTL_OFFSET 0x00000138
#define BLE_PERADVLCNTL_INDEX  0x0000004E
#define BLE_PERADVLCNTL_RESET  0x00000000

__INLINE uint32_t ble_peradvlcntl_get(void)
{
    return REG_BLE_RD(BLE_PERADVLCNTL_ADDR);
}

__INLINE void ble_peradvlcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_PERADVLCNTL_ADDR, value);
}

// field definitions
#define BLE_PERADVLNBDEV_MASK     ((uint32_t)0x00FF0000)
#define BLE_PERADVLNBDEV_LSB      16
#define BLE_PERADVLNBDEV_WIDTH    ((uint32_t)0x00000008)
#define BLE_PERADVLBASEPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_PERADVLBASEPTR_LSB    0
#define BLE_PERADVLBASEPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_PERADVLNBDEV_RST      0x0
#define BLE_PERADVLBASEPTR_RST    0x0

__INLINE void ble_peradvlcntl_pack(uint8_t peradvlnbdev, uint16_t peradvlbaseptr)
{
    ASSERT_ERR((((uint32_t)peradvlnbdev << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)peradvlbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_PERADVLCNTL_ADDR, ((uint32_t)peradvlnbdev << 16) | ((uint32_t)peradvlbaseptr << 0));
}

__INLINE void ble_peradvlcntl_unpack(uint8_t *peradvlnbdev, uint16_t *peradvlbaseptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_PERADVLCNTL_ADDR);

    *peradvlnbdev = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *peradvlbaseptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t ble_peradvlcntl_peradvlnbdev_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_PERADVLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_peradvlcntl_peradvlnbdev_setf(uint8_t peradvlnbdev)
{
    ASSERT_ERR((((uint32_t)peradvlnbdev << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_PERADVLCNTL_ADDR, (REG_BLE_RD(BLE_PERADVLCNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)peradvlnbdev << 16));
}

__INLINE uint16_t ble_peradvlcntl_peradvlbaseptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_PERADVLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_peradvlcntl_peradvlbaseptr_setf(uint16_t peradvlbaseptr)
{
    ASSERT_ERR((((uint32_t)peradvlbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_PERADVLCNTL_ADDR, (REG_BLE_RD(BLE_PERADVLCNTL_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)peradvlbaseptr << 0));
}

/**
 * @brief PERADVLCURRENTPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00    PERADVLCURRENTPTR   0x0
 * </pre>
 */
#define BLE_PERADVLCURRENTPTR_ADDR   0x4010013C
#define BLE_PERADVLCURRENTPTR_OFFSET 0x0000013C
#define BLE_PERADVLCURRENTPTR_INDEX  0x0000004F
#define BLE_PERADVLCURRENTPTR_RESET  0x00000000

__INLINE uint32_t ble_peradvlcurrentptr_get(void)
{
    return REG_BLE_RD(BLE_PERADVLCURRENTPTR_ADDR);
}

__INLINE void ble_peradvlcurrentptr_set(uint32_t value)
{
    REG_BLE_WR(BLE_PERADVLCURRENTPTR_ADDR, value);
}

// field definitions
#define BLE_PERADVLCURRENTPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_PERADVLCURRENTPTR_LSB    0
#define BLE_PERADVLCURRENTPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_PERADVLCURRENTPTR_RST    0x0

__INLINE uint16_t ble_peradvlcurrentptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_PERADVLCURRENTPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_peradvlcurrentptr_setf(uint16_t peradvlcurrentptr)
{
    ASSERT_ERR((((uint32_t)peradvlcurrentptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_PERADVLCURRENTPTR_ADDR, (uint32_t)peradvlcurrentptr << 0);
}

/**
 * @brief ADILCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16                NBADI   0x0
 *  15:00          ADILBASEPTR   0x0
 * </pre>
 */
#define BLE_ADILCNTL_ADDR   0x40100140
#define BLE_ADILCNTL_OFFSET 0x00000140
#define BLE_ADILCNTL_INDEX  0x00000050
#define BLE_ADILCNTL_RESET  0x00000000

__INLINE uint32_t ble_adilcntl_get(void)
{
    return REG_BLE_RD(BLE_ADILCNTL_ADDR);
}

__INLINE void ble_adilcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_ADILCNTL_ADDR, value);
}

// field definitions
#define BLE_NBADI_MASK         ((uint32_t)0x00FF0000)
#define BLE_NBADI_LSB          16
#define BLE_NBADI_WIDTH        ((uint32_t)0x00000008)
#define BLE_ADILBASEPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ADILBASEPTR_LSB    0
#define BLE_ADILBASEPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_NBADI_RST          0x0
#define BLE_ADILBASEPTR_RST    0x0

__INLINE void ble_adilcntl_pack(uint8_t nbadi, uint16_t adilbaseptr)
{
    ASSERT_ERR((((uint32_t)nbadi << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)adilbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ADILCNTL_ADDR, ((uint32_t)nbadi << 16) | ((uint32_t)adilbaseptr << 0));
}

__INLINE void ble_adilcntl_unpack(uint8_t *nbadi, uint16_t *adilbaseptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADILCNTL_ADDR);

    *nbadi = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *adilbaseptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t ble_adilcntl_nbadi_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADILCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_adilcntl_nbadi_setf(uint8_t nbadi)
{
    ASSERT_ERR((((uint32_t)nbadi << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_ADILCNTL_ADDR, (REG_BLE_RD(BLE_ADILCNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)nbadi << 16));
}

__INLINE uint16_t ble_adilcntl_adilbaseptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADILCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_adilcntl_adilbaseptr_setf(uint16_t adilbaseptr)
{
    ASSERT_ERR((((uint32_t)adilbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ADILCNTL_ADDR, (REG_BLE_RD(BLE_ADILCNTL_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)adilbaseptr << 0));
}

/**
 * @brief ADILCURRENTPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       ADILCURRENTPTR   0x0
 * </pre>
 */
#define BLE_ADILCURRENTPTR_ADDR   0x40100144
#define BLE_ADILCURRENTPTR_OFFSET 0x00000144
#define BLE_ADILCURRENTPTR_INDEX  0x00000051
#define BLE_ADILCURRENTPTR_RESET  0x00000000

__INLINE uint32_t ble_adilcurrentptr_get(void)
{
    return REG_BLE_RD(BLE_ADILCURRENTPTR_ADDR);
}

__INLINE void ble_adilcurrentptr_set(uint32_t value)
{
    REG_BLE_WR(BLE_ADILCURRENTPTR_ADDR, value);
}

// field definitions
#define BLE_ADILCURRENTPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ADILCURRENTPTR_LSB    0
#define BLE_ADILCURRENTPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_ADILCURRENTPTR_RST    0x0

__INLINE uint16_t ble_adilcurrentptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ADILCURRENTPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_adilcurrentptr_setf(uint16_t adilcurrentptr)
{
    ASSERT_ERR((((uint32_t)adilcurrentptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ADILCURRENTPTR_ADDR, (uint32_t)adilcurrentptr << 0);
}

/**
 * @brief SEARCH_TIMEOUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  05:00       SEARCH_TIMEOUT   0x10
 * </pre>
 */
#define BLE_SEARCH_TIMEOUT_ADDR   0x40100148
#define BLE_SEARCH_TIMEOUT_OFFSET 0x00000148
#define BLE_SEARCH_TIMEOUT_INDEX  0x00000052
#define BLE_SEARCH_TIMEOUT_RESET  0x00000010

__INLINE uint32_t ble_search_timeout_get(void)
{
    return REG_BLE_RD(BLE_SEARCH_TIMEOUT_ADDR);
}

__INLINE void ble_search_timeout_set(uint32_t value)
{
    REG_BLE_WR(BLE_SEARCH_TIMEOUT_ADDR, value);
}

// field definitions
#define BLE_SEARCH_TIMEOUT_MASK   ((uint32_t)0x0000003F)
#define BLE_SEARCH_TIMEOUT_LSB    0
#define BLE_SEARCH_TIMEOUT_WIDTH  ((uint32_t)0x00000006)

#define BLE_SEARCH_TIMEOUT_RST    0x10

__INLINE uint8_t ble_search_timeout_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_SEARCH_TIMEOUT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000003F)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_search_timeout_setf(uint8_t searchtimeout)
{
    ASSERT_ERR((((uint32_t)searchtimeout << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_BLE_WR(BLE_SEARCH_TIMEOUT_ADDR, (uint32_t)searchtimeout << 0);
}

/**
 * @brief COEXIFCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
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
#define BLE_COEXIFCNTL0_ADDR   0x40100150
#define BLE_COEXIFCNTL0_OFFSET 0x00000150
#define BLE_COEXIFCNTL0_INDEX  0x00000054
#define BLE_COEXIFCNTL0_RESET  0x00000010

__INLINE uint32_t ble_coexifcntl0_get(void)
{
    return REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
}

__INLINE void ble_coexifcntl0_set(uint32_t value)
{
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, value);
}

// field definitions
#define BLE_MWSSCANFREQMSK_MASK   ((uint32_t)0x00300000)
#define BLE_MWSSCANFREQMSK_LSB    20
#define BLE_MWSSCANFREQMSK_WIDTH  ((uint32_t)0x00000002)
#define BLE_WLCRXPRIOMODE_MASK    ((uint32_t)0x000C0000)
#define BLE_WLCRXPRIOMODE_LSB     18
#define BLE_WLCRXPRIOMODE_WIDTH   ((uint32_t)0x00000002)
#define BLE_WLCTXPRIOMODE_MASK    ((uint32_t)0x00030000)
#define BLE_WLCTXPRIOMODE_LSB     16
#define BLE_WLCTXPRIOMODE_WIDTH   ((uint32_t)0x00000002)
#define BLE_MWSTXFRQMSK_MASK      ((uint32_t)0x0000C000)
#define BLE_MWSTXFRQMSK_LSB       14
#define BLE_MWSTXFRQMSK_WIDTH     ((uint32_t)0x00000002)
#define BLE_MWSRXFRQMSK_MASK      ((uint32_t)0x00003000)
#define BLE_MWSRXFRQMSK_LSB       12
#define BLE_MWSRXFRQMSK_WIDTH     ((uint32_t)0x00000002)
#define BLE_MWSTXMSK_MASK         ((uint32_t)0x00000C00)
#define BLE_MWSTXMSK_LSB          10
#define BLE_MWSTXMSK_WIDTH        ((uint32_t)0x00000002)
#define BLE_MWSRXMSK_MASK         ((uint32_t)0x00000300)
#define BLE_MWSRXMSK_LSB          8
#define BLE_MWSRXMSK_WIDTH        ((uint32_t)0x00000002)
#define BLE_WLANTXMSK_MASK        ((uint32_t)0x000000C0)
#define BLE_WLANTXMSK_LSB         6
#define BLE_WLANTXMSK_WIDTH       ((uint32_t)0x00000002)
#define BLE_WLANRXMSK_MASK        ((uint32_t)0x00000030)
#define BLE_WLANRXMSK_LSB         4
#define BLE_WLANRXMSK_WIDTH       ((uint32_t)0x00000002)
#define BLE_MWSWCI_EN_BIT         ((uint32_t)0x00000008)
#define BLE_MWSWCI_EN_POS         3
#define BLE_MWSCOEX_EN_BIT        ((uint32_t)0x00000004)
#define BLE_MWSCOEX_EN_POS        2
#define BLE_SYNCGEN_EN_BIT        ((uint32_t)0x00000002)
#define BLE_SYNCGEN_EN_POS        1
#define BLE_WLANCOEX_EN_BIT       ((uint32_t)0x00000001)
#define BLE_WLANCOEX_EN_POS       0

#define BLE_MWSSCANFREQMSK_RST    0x0
#define BLE_WLCRXPRIOMODE_RST     0x0
#define BLE_WLCTXPRIOMODE_RST     0x0
#define BLE_MWSTXFRQMSK_RST       0x0
#define BLE_MWSRXFRQMSK_RST       0x0
#define BLE_MWSTXMSK_RST          0x0
#define BLE_MWSRXMSK_RST          0x0
#define BLE_WLANTXMSK_RST         0x0
#define BLE_WLANRXMSK_RST         0x1
#define BLE_MWSWCI_EN_RST         0x0
#define BLE_MWSCOEX_EN_RST        0x0
#define BLE_SYNCGEN_EN_RST        0x0
#define BLE_WLANCOEX_EN_RST       0x0

__INLINE void ble_coexifcntl0_pack(uint8_t mwsscanfreqmsk, uint8_t wlcrxpriomode, uint8_t wlctxpriomode, uint8_t mwstxfrqmsk, uint8_t mwsrxfrqmsk, uint8_t mwstxmsk, uint8_t mwsrxmsk, uint8_t wlantxmsk, uint8_t wlanrxmsk, uint8_t mwswcien, uint8_t mwscoexen, uint8_t syncgenen, uint8_t wlancoexen)
{
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
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, ((uint32_t)mwsscanfreqmsk << 20) | ((uint32_t)wlcrxpriomode << 18) | ((uint32_t)wlctxpriomode << 16) | ((uint32_t)mwstxfrqmsk << 14) | ((uint32_t)mwsrxfrqmsk << 12) | ((uint32_t)mwstxmsk << 10) | ((uint32_t)mwsrxmsk << 8) | ((uint32_t)wlantxmsk << 6) | ((uint32_t)wlanrxmsk << 4) | ((uint32_t)mwswcien << 3) | ((uint32_t)mwscoexen << 2) | ((uint32_t)syncgenen << 1) | ((uint32_t)wlancoexen << 0));
}

__INLINE void ble_coexifcntl0_unpack(uint8_t *mwsscanfreqmsk, uint8_t *wlcrxpriomode, uint8_t *wlctxpriomode, uint8_t *mwstxfrqmsk, uint8_t *mwsrxfrqmsk, uint8_t *mwstxmsk, uint8_t *mwsrxmsk, uint8_t *wlantxmsk, uint8_t *wlanrxmsk, uint8_t *mwswcien, uint8_t *mwscoexen, uint8_t *syncgenen, uint8_t *wlancoexen)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);

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

__INLINE uint8_t ble_coexifcntl0_mwsscanfreqmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void ble_coexifcntl0_mwsscanfreqmsk_setf(uint8_t mwsscanfreqmsk)
{
    ASSERT_ERR((((uint32_t)mwsscanfreqmsk << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)mwsscanfreqmsk << 20));
}

__INLINE uint8_t ble_coexifcntl0_wlcrxpriomode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void ble_coexifcntl0_wlcrxpriomode_setf(uint8_t wlcrxpriomode)
{
    ASSERT_ERR((((uint32_t)wlcrxpriomode << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)wlcrxpriomode << 18));
}

__INLINE uint8_t ble_coexifcntl0_wlctxpriomode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void ble_coexifcntl0_wlctxpriomode_setf(uint8_t wlctxpriomode)
{
    ASSERT_ERR((((uint32_t)wlctxpriomode << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)wlctxpriomode << 16));
}

__INLINE uint8_t ble_coexifcntl0_mwstxfrqmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void ble_coexifcntl0_mwstxfrqmsk_setf(uint8_t mwstxfrqmsk)
{
    ASSERT_ERR((((uint32_t)mwstxfrqmsk << 14) & ~((uint32_t)0x0000C000)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)mwstxfrqmsk << 14));
}

__INLINE uint8_t ble_coexifcntl0_mwsrxfrqmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void ble_coexifcntl0_mwsrxfrqmsk_setf(uint8_t mwsrxfrqmsk)
{
    ASSERT_ERR((((uint32_t)mwsrxfrqmsk << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)mwsrxfrqmsk << 12));
}

__INLINE uint8_t ble_coexifcntl0_mwstxmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void ble_coexifcntl0_mwstxmsk_setf(uint8_t mwstxmsk)
{
    ASSERT_ERR((((uint32_t)mwstxmsk << 10) & ~((uint32_t)0x00000C00)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)mwstxmsk << 10));
}

__INLINE uint8_t ble_coexifcntl0_mwsrxmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void ble_coexifcntl0_mwsrxmsk_setf(uint8_t mwsrxmsk)
{
    ASSERT_ERR((((uint32_t)mwsrxmsk << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)mwsrxmsk << 8));
}

__INLINE uint8_t ble_coexifcntl0_wlantxmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000C0)) >> 6);
}

__INLINE void ble_coexifcntl0_wlantxmsk_setf(uint8_t wlantxmsk)
{
    ASSERT_ERR((((uint32_t)wlantxmsk << 6) & ~((uint32_t)0x000000C0)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x000000C0)) | ((uint32_t)wlantxmsk << 6));
}

__INLINE uint8_t ble_coexifcntl0_wlanrxmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void ble_coexifcntl0_wlanrxmsk_setf(uint8_t wlanrxmsk)
{
    ASSERT_ERR((((uint32_t)wlanrxmsk << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)wlanrxmsk << 4));
}

__INLINE uint8_t ble_coexifcntl0_mwswci_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_coexifcntl0_mwswci_en_setf(uint8_t mwswcien)
{
    ASSERT_ERR((((uint32_t)mwswcien << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)mwswcien << 3));
}

__INLINE uint8_t ble_coexifcntl0_mwscoex_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_coexifcntl0_mwscoex_en_setf(uint8_t mwscoexen)
{
    ASSERT_ERR((((uint32_t)mwscoexen << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)mwscoexen << 2));
}

__INLINE uint8_t ble_coexifcntl0_syncgen_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ble_coexifcntl0_syncgen_en_setf(uint8_t syncgenen)
{
    ASSERT_ERR((((uint32_t)syncgenen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)syncgenen << 1));
}

__INLINE uint8_t ble_coexifcntl0_wlancoex_en_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_coexifcntl0_wlancoex_en_setf(uint8_t wlancoexen)
{
    ASSERT_ERR((((uint32_t)wlancoexen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL0_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)wlancoexen << 0));
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
#define BLE_COEXIFCNTL1_ADDR   0x40100154
#define BLE_COEXIFCNTL1_OFFSET 0x00000154
#define BLE_COEXIFCNTL1_INDEX  0x00000055
#define BLE_COEXIFCNTL1_RESET  0x00000000

__INLINE uint32_t ble_coexifcntl1_get(void)
{
    return REG_BLE_RD(BLE_COEXIFCNTL1_ADDR);
}

__INLINE void ble_coexifcntl1_set(uint32_t value)
{
    REG_BLE_WR(BLE_COEXIFCNTL1_ADDR, value);
}

// field definitions
#define BLE_WLCPRXTHR_MASK      ((uint32_t)0x1F000000)
#define BLE_WLCPRXTHR_LSB       24
#define BLE_WLCPRXTHR_WIDTH     ((uint32_t)0x00000005)
#define BLE_WLCPTXTHR_MASK      ((uint32_t)0x001F0000)
#define BLE_WLCPTXTHR_LSB       16
#define BLE_WLCPTXTHR_WIDTH     ((uint32_t)0x00000005)
#define BLE_WLCPDURATION_MASK   ((uint32_t)0x00007F00)
#define BLE_WLCPDURATION_LSB    8
#define BLE_WLCPDURATION_WIDTH  ((uint32_t)0x00000007)
#define BLE_WLCPDELAY_MASK      ((uint32_t)0x0000007F)
#define BLE_WLCPDELAY_LSB       0
#define BLE_WLCPDELAY_WIDTH     ((uint32_t)0x00000007)

#define BLE_WLCPRXTHR_RST       0x0
#define BLE_WLCPTXTHR_RST       0x0
#define BLE_WLCPDURATION_RST    0x0
#define BLE_WLCPDELAY_RST       0x0

__INLINE void ble_coexifcntl1_pack(uint8_t wlcprxthr, uint8_t wlcptxthr, uint8_t wlcpduration, uint8_t wlcpdelay)
{
    ASSERT_ERR((((uint32_t)wlcprxthr << 24) & ~((uint32_t)0x1F000000)) == 0);
    ASSERT_ERR((((uint32_t)wlcptxthr << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)wlcpduration << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)wlcpdelay << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL1_ADDR, ((uint32_t)wlcprxthr << 24) | ((uint32_t)wlcptxthr << 16) | ((uint32_t)wlcpduration << 8) | ((uint32_t)wlcpdelay << 0));
}

__INLINE void ble_coexifcntl1_unpack(uint8_t *wlcprxthr, uint8_t *wlcptxthr, uint8_t *wlcpduration, uint8_t *wlcpdelay)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL1_ADDR);

    *wlcprxthr = (localVal & ((uint32_t)0x1F000000)) >> 24;
    *wlcptxthr = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *wlcpduration = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *wlcpdelay = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t ble_coexifcntl1_wlcprxthr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x1F000000)) >> 24);
}

__INLINE void ble_coexifcntl1_wlcprxthr_setf(uint8_t wlcprxthr)
{
    ASSERT_ERR((((uint32_t)wlcprxthr << 24) & ~((uint32_t)0x1F000000)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL1_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL1_ADDR) & ~((uint32_t)0x1F000000)) | ((uint32_t)wlcprxthr << 24));
}

__INLINE uint8_t ble_coexifcntl1_wlcptxthr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void ble_coexifcntl1_wlcptxthr_setf(uint8_t wlcptxthr)
{
    ASSERT_ERR((((uint32_t)wlcptxthr << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL1_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL1_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)wlcptxthr << 16));
}

__INLINE uint8_t ble_coexifcntl1_wlcpduration_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void ble_coexifcntl1_wlcpduration_setf(uint8_t wlcpduration)
{
    ASSERT_ERR((((uint32_t)wlcpduration << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL1_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL1_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)wlcpduration << 8));
}

__INLINE uint8_t ble_coexifcntl1_wlcpdelay_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void ble_coexifcntl1_wlcpdelay_setf(uint8_t wlcpdelay)
{
    ASSERT_ERR((((uint32_t)wlcpdelay << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL1_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL1_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)wlcpdelay << 0));
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
#define BLE_COEXIFCNTL2_ADDR   0x40100158
#define BLE_COEXIFCNTL2_OFFSET 0x00000158
#define BLE_COEXIFCNTL2_INDEX  0x00000056
#define BLE_COEXIFCNTL2_RESET  0x00000000

__INLINE uint32_t ble_coexifcntl2_get(void)
{
    return REG_BLE_RD(BLE_COEXIFCNTL2_ADDR);
}

__INLINE void ble_coexifcntl2_set(uint32_t value)
{
    REG_BLE_WR(BLE_COEXIFCNTL2_ADDR, value);
}

// field definitions
#define BLE_RX_ANT_DELAY_MASK   ((uint32_t)0x00000F00)
#define BLE_RX_ANT_DELAY_LSB    8
#define BLE_RX_ANT_DELAY_WIDTH  ((uint32_t)0x00000004)
#define BLE_TX_ANT_DELAY_MASK   ((uint32_t)0x0000000F)
#define BLE_TX_ANT_DELAY_LSB    0
#define BLE_TX_ANT_DELAY_WIDTH  ((uint32_t)0x00000004)

#define BLE_RX_ANT_DELAY_RST    0x0
#define BLE_TX_ANT_DELAY_RST    0x0

__INLINE void ble_coexifcntl2_pack(uint8_t rxantdelay, uint8_t txantdelay)
{
    ASSERT_ERR((((uint32_t)rxantdelay << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)txantdelay << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL2_ADDR, ((uint32_t)rxantdelay << 8) | ((uint32_t)txantdelay << 0));
}

__INLINE void ble_coexifcntl2_unpack(uint8_t *rxantdelay, uint8_t *txantdelay)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL2_ADDR);

    *rxantdelay = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *txantdelay = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ble_coexifcntl2_rx_ant_delay_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void ble_coexifcntl2_rx_ant_delay_setf(uint8_t rxantdelay)
{
    ASSERT_ERR((((uint32_t)rxantdelay << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL2_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL2_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)rxantdelay << 8));
}

__INLINE uint8_t ble_coexifcntl2_tx_ant_delay_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_COEXIFCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ble_coexifcntl2_tx_ant_delay_setf(uint8_t txantdelay)
{
    ASSERT_ERR((((uint32_t)txantdelay << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_COEXIFCNTL2_ADDR, (REG_BLE_RD(BLE_COEXIFCNTL2_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)txantdelay << 0));
}

/**
 * @brief BLEMPRIO0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28                BLEM7   0x3
 *  27:24                BLEM6   0x4
 *  23:20                BLEM5   0x8
 *  19:16                BLEM4   0x9
 *  15:12                BLEM3   0xA
 *  11:08                BLEM2   0xD
 *  07:04                BLEM1   0xE
 *  03:00                BLEM0   0xF
 * </pre>
 */
#define BLE_BLEMPRIO0_ADDR   0x4010015C
#define BLE_BLEMPRIO0_OFFSET 0x0000015C
#define BLE_BLEMPRIO0_INDEX  0x00000057
#define BLE_BLEMPRIO0_RESET  0x3489ADEF

__INLINE uint32_t ble_blemprio0_get(void)
{
    return REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
}

__INLINE void ble_blemprio0_set(uint32_t value)
{
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, value);
}

// field definitions
#define BLE_BLEM7_MASK   ((uint32_t)0xF0000000)
#define BLE_BLEM7_LSB    28
#define BLE_BLEM7_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM6_MASK   ((uint32_t)0x0F000000)
#define BLE_BLEM6_LSB    24
#define BLE_BLEM6_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM5_MASK   ((uint32_t)0x00F00000)
#define BLE_BLEM5_LSB    20
#define BLE_BLEM5_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM4_MASK   ((uint32_t)0x000F0000)
#define BLE_BLEM4_LSB    16
#define BLE_BLEM4_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM3_MASK   ((uint32_t)0x0000F000)
#define BLE_BLEM3_LSB    12
#define BLE_BLEM3_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM2_MASK   ((uint32_t)0x00000F00)
#define BLE_BLEM2_LSB    8
#define BLE_BLEM2_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM1_MASK   ((uint32_t)0x000000F0)
#define BLE_BLEM1_LSB    4
#define BLE_BLEM1_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM0_MASK   ((uint32_t)0x0000000F)
#define BLE_BLEM0_LSB    0
#define BLE_BLEM0_WIDTH  ((uint32_t)0x00000004)

#define BLE_BLEM7_RST    0x3
#define BLE_BLEM6_RST    0x4
#define BLE_BLEM5_RST    0x8
#define BLE_BLEM4_RST    0x9
#define BLE_BLEM3_RST    0xA
#define BLE_BLEM2_RST    0xD
#define BLE_BLEM1_RST    0xE
#define BLE_BLEM0_RST    0xF

__INLINE void ble_blemprio0_pack(uint8_t blem7, uint8_t blem6, uint8_t blem5, uint8_t blem4, uint8_t blem3, uint8_t blem2, uint8_t blem1, uint8_t blem0)
{
    ASSERT_ERR((((uint32_t)blem7 << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)blem6 << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)blem5 << 20) & ~((uint32_t)0x00F00000)) == 0);
    ASSERT_ERR((((uint32_t)blem4 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)blem3 << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)blem2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)blem1 << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)blem0 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, ((uint32_t)blem7 << 28) | ((uint32_t)blem6 << 24) | ((uint32_t)blem5 << 20) | ((uint32_t)blem4 << 16) | ((uint32_t)blem3 << 12) | ((uint32_t)blem2 << 8) | ((uint32_t)blem1 << 4) | ((uint32_t)blem0 << 0));
}

__INLINE void ble_blemprio0_unpack(uint8_t *blem7, uint8_t *blem6, uint8_t *blem5, uint8_t *blem4, uint8_t *blem3, uint8_t *blem2, uint8_t *blem1, uint8_t *blem0)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);

    *blem7 = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *blem6 = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *blem5 = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *blem4 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *blem3 = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *blem2 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *blem1 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *blem0 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ble_blemprio0_blem7_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void ble_blemprio0_blem7_setf(uint8_t blem7)
{
    ASSERT_ERR((((uint32_t)blem7 << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)blem7 << 28));
}

__INLINE uint8_t ble_blemprio0_blem6_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void ble_blemprio0_blem6_setf(uint8_t blem6)
{
    ASSERT_ERR((((uint32_t)blem6 << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)blem6 << 24));
}

__INLINE uint8_t ble_blemprio0_blem5_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void ble_blemprio0_blem5_setf(uint8_t blem5)
{
    ASSERT_ERR((((uint32_t)blem5 << 20) & ~((uint32_t)0x00F00000)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)blem5 << 20));
}

__INLINE uint8_t ble_blemprio0_blem4_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void ble_blemprio0_blem4_setf(uint8_t blem4)
{
    ASSERT_ERR((((uint32_t)blem4 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)blem4 << 16));
}

__INLINE uint8_t ble_blemprio0_blem3_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void ble_blemprio0_blem3_setf(uint8_t blem3)
{
    ASSERT_ERR((((uint32_t)blem3 << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)blem3 << 12));
}

__INLINE uint8_t ble_blemprio0_blem2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void ble_blemprio0_blem2_setf(uint8_t blem2)
{
    ASSERT_ERR((((uint32_t)blem2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)blem2 << 8));
}

__INLINE uint8_t ble_blemprio0_blem1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void ble_blemprio0_blem1_setf(uint8_t blem1)
{
    ASSERT_ERR((((uint32_t)blem1 << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)blem1 << 4));
}

__INLINE uint8_t ble_blemprio0_blem0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO0_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ble_blemprio0_blem0_setf(uint8_t blem0)
{
    ASSERT_ERR((((uint32_t)blem0 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO0_ADDR, (REG_BLE_RD(BLE_BLEMPRIO0_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)blem0 << 0));
}

/**
 * @brief BLEMPRIO1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28          BLEMDEFAULT   0x3
 *  07:04                BLEM9   0xD
 *  03:00                BLEM8   0xC
 * </pre>
 */
#define BLE_BLEMPRIO1_ADDR   0x40100160
#define BLE_BLEMPRIO1_OFFSET 0x00000160
#define BLE_BLEMPRIO1_INDEX  0x00000058
#define BLE_BLEMPRIO1_RESET  0x300000DC

__INLINE uint32_t ble_blemprio1_get(void)
{
    return REG_BLE_RD(BLE_BLEMPRIO1_ADDR);
}

__INLINE void ble_blemprio1_set(uint32_t value)
{
    REG_BLE_WR(BLE_BLEMPRIO1_ADDR, value);
}

// field definitions
#define BLE_BLEMDEFAULT_MASK   ((uint32_t)0xF0000000)
#define BLE_BLEMDEFAULT_LSB    28
#define BLE_BLEMDEFAULT_WIDTH  ((uint32_t)0x00000004)
#define BLE_BLEM9_MASK         ((uint32_t)0x000000F0)
#define BLE_BLEM9_LSB          4
#define BLE_BLEM9_WIDTH        ((uint32_t)0x00000004)
#define BLE_BLEM8_MASK         ((uint32_t)0x0000000F)
#define BLE_BLEM8_LSB          0
#define BLE_BLEM8_WIDTH        ((uint32_t)0x00000004)

#define BLE_BLEMDEFAULT_RST    0x3
#define BLE_BLEM9_RST          0xD
#define BLE_BLEM8_RST          0xC

__INLINE void ble_blemprio1_pack(uint8_t blemdefault, uint8_t blem9, uint8_t blem8)
{
    ASSERT_ERR((((uint32_t)blemdefault << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)blem9 << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)blem8 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO1_ADDR, ((uint32_t)blemdefault << 28) | ((uint32_t)blem9 << 4) | ((uint32_t)blem8 << 0));
}

__INLINE void ble_blemprio1_unpack(uint8_t *blemdefault, uint8_t *blem9, uint8_t *blem8)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO1_ADDR);

    *blemdefault = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *blem9 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *blem8 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ble_blemprio1_blemdefault_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void ble_blemprio1_blemdefault_setf(uint8_t blemdefault)
{
    ASSERT_ERR((((uint32_t)blemdefault << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO1_ADDR, (REG_BLE_RD(BLE_BLEMPRIO1_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)blemdefault << 28));
}

__INLINE uint8_t ble_blemprio1_blem9_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void ble_blemprio1_blem9_setf(uint8_t blem9)
{
    ASSERT_ERR((((uint32_t)blem9 << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO1_ADDR, (REG_BLE_RD(BLE_BLEMPRIO1_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)blem9 << 4));
}

__INLINE uint8_t ble_blemprio1_blem8_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_BLEMPRIO1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ble_blemprio1_blem8_setf(uint8_t blem8)
{
    ASSERT_ERR((((uint32_t)blem8 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_BLE_WR(BLE_BLEMPRIO1_ADDR, (REG_BLE_RD(BLE_BLEMPRIO1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)blem8 << 0));
}

/**
 * @brief RALCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16             RALNBDEV   0x0
 *  15:00           RALBASEPTR   0x0
 * </pre>
 */
#define BLE_RALCNTL_ADDR   0x40100170
#define BLE_RALCNTL_OFFSET 0x00000170
#define BLE_RALCNTL_INDEX  0x0000005C
#define BLE_RALCNTL_RESET  0x00000000

__INLINE uint32_t ble_ralcntl_get(void)
{
    return REG_BLE_RD(BLE_RALCNTL_ADDR);
}

__INLINE void ble_ralcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_RALCNTL_ADDR, value);
}

// field definitions
#define BLE_RALNBDEV_MASK     ((uint32_t)0x00FF0000)
#define BLE_RALNBDEV_LSB      16
#define BLE_RALNBDEV_WIDTH    ((uint32_t)0x00000008)
#define BLE_RALBASEPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_RALBASEPTR_LSB    0
#define BLE_RALBASEPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_RALNBDEV_RST      0x0
#define BLE_RALBASEPTR_RST    0x0

__INLINE void ble_ralcntl_pack(uint8_t ralnbdev, uint16_t ralbaseptr)
{
    ASSERT_ERR((((uint32_t)ralnbdev << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)ralbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_RALCNTL_ADDR, ((uint32_t)ralnbdev << 16) | ((uint32_t)ralbaseptr << 0));
}

__INLINE void ble_ralcntl_unpack(uint8_t *ralnbdev, uint16_t *ralbaseptr)
{
    uint32_t localVal = REG_BLE_RD(BLE_RALCNTL_ADDR);

    *ralnbdev = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *ralbaseptr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t ble_ralcntl_ralnbdev_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RALCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_ralcntl_ralnbdev_setf(uint8_t ralnbdev)
{
    ASSERT_ERR((((uint32_t)ralnbdev << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_RALCNTL_ADDR, (REG_BLE_RD(BLE_RALCNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)ralnbdev << 16));
}

__INLINE uint16_t ble_ralcntl_ralbaseptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RALCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_ralcntl_ralbaseptr_setf(uint16_t ralbaseptr)
{
    ASSERT_ERR((((uint32_t)ralbaseptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_RALCNTL_ADDR, (REG_BLE_RD(BLE_RALCNTL_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)ralbaseptr << 0));
}

/**
 * @brief RALCURRENTPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00        RALCURRENTPTR   0x0
 * </pre>
 */
#define BLE_RALCURRENTPTR_ADDR   0x40100174
#define BLE_RALCURRENTPTR_OFFSET 0x00000174
#define BLE_RALCURRENTPTR_INDEX  0x0000005D
#define BLE_RALCURRENTPTR_RESET  0x00000000

__INLINE uint32_t ble_ralcurrentptr_get(void)
{
    return REG_BLE_RD(BLE_RALCURRENTPTR_ADDR);
}

__INLINE void ble_ralcurrentptr_set(uint32_t value)
{
    REG_BLE_WR(BLE_RALCURRENTPTR_ADDR, value);
}

// field definitions
#define BLE_RALCURRENTPTR_MASK   ((uint32_t)0x0000FFFF)
#define BLE_RALCURRENTPTR_LSB    0
#define BLE_RALCURRENTPTR_WIDTH  ((uint32_t)0x00000010)

#define BLE_RALCURRENTPTR_RST    0x0

__INLINE uint16_t ble_ralcurrentptr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RALCURRENTPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_ralcurrentptr_setf(uint16_t ralcurrentptr)
{
    ASSERT_ERR((((uint32_t)ralcurrentptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_RALCURRENTPTR_ADDR, (uint32_t)ralcurrentptr << 0);
}

/**
 * @brief RAL_LOCAL_RND register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            LRND_INIT   0
 *  21:00             LRND_VAL   0x3F0F0F
 * </pre>
 */
#define BLE_RAL_LOCAL_RND_ADDR   0x40100178
#define BLE_RAL_LOCAL_RND_OFFSET 0x00000178
#define BLE_RAL_LOCAL_RND_INDEX  0x0000005E
#define BLE_RAL_LOCAL_RND_RESET  0x003F0F0F

__INLINE uint32_t ble_ral_local_rnd_get(void)
{
    return REG_BLE_RD(BLE_RAL_LOCAL_RND_ADDR);
}

__INLINE void ble_ral_local_rnd_set(uint32_t value)
{
    REG_BLE_WR(BLE_RAL_LOCAL_RND_ADDR, value);
}

// field definitions
#define BLE_LRND_INIT_BIT    ((uint32_t)0x80000000)
#define BLE_LRND_INIT_POS    31
#define BLE_LRND_VAL_MASK    ((uint32_t)0x003FFFFF)
#define BLE_LRND_VAL_LSB     0
#define BLE_LRND_VAL_WIDTH   ((uint32_t)0x00000016)

#define BLE_LRND_INIT_RST    0x0
#define BLE_LRND_VAL_RST     0x3F0F0F

__INLINE void ble_ral_local_rnd_pack(uint8_t lrndinit, uint32_t lrndval)
{
    ASSERT_ERR((((uint32_t)lrndinit << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)lrndval << 0) & ~((uint32_t)0x003FFFFF)) == 0);
    REG_BLE_WR(BLE_RAL_LOCAL_RND_ADDR, ((uint32_t)lrndinit << 31) | ((uint32_t)lrndval << 0));
}

__INLINE void ble_ral_local_rnd_unpack(uint8_t *lrndinit, uint32_t *lrndval)
{
    uint32_t localVal = REG_BLE_RD(BLE_RAL_LOCAL_RND_ADDR);

    *lrndinit = (localVal & ((uint32_t)0x80000000)) >> 31;
    *lrndval = (localVal & ((uint32_t)0x003FFFFF)) >> 0;
}

__INLINE uint8_t ble_ral_local_rnd_lrnd_init_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RAL_LOCAL_RND_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_ral_local_rnd_lrnd_init_setf(uint8_t lrndinit)
{
    ASSERT_ERR((((uint32_t)lrndinit << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_RAL_LOCAL_RND_ADDR, (REG_BLE_RD(BLE_RAL_LOCAL_RND_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)lrndinit << 31));
}

__INLINE uint32_t ble_ral_local_rnd_lrnd_val_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RAL_LOCAL_RND_ADDR);
    return ((localVal & ((uint32_t)0x003FFFFF)) >> 0);
}

__INLINE void ble_ral_local_rnd_lrnd_val_setf(uint32_t lrndval)
{
    ASSERT_ERR((((uint32_t)lrndval << 0) & ~((uint32_t)0x003FFFFF)) == 0);
    REG_BLE_WR(BLE_RAL_LOCAL_RND_ADDR, (REG_BLE_RD(BLE_RAL_LOCAL_RND_ADDR) & ~((uint32_t)0x003FFFFF)) | ((uint32_t)lrndval << 0));
}

/**
 * @brief RAL_PEER_RND register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            PRND_INIT   0
 *  21:00             PRND_VAL   0x30F0F0
 * </pre>
 */
#define BLE_RAL_PEER_RND_ADDR   0x4010017C
#define BLE_RAL_PEER_RND_OFFSET 0x0000017C
#define BLE_RAL_PEER_RND_INDEX  0x0000005F
#define BLE_RAL_PEER_RND_RESET  0x0030F0F0

__INLINE uint32_t ble_ral_peer_rnd_get(void)
{
    return REG_BLE_RD(BLE_RAL_PEER_RND_ADDR);
}

__INLINE void ble_ral_peer_rnd_set(uint32_t value)
{
    REG_BLE_WR(BLE_RAL_PEER_RND_ADDR, value);
}

// field definitions
#define BLE_PRND_INIT_BIT    ((uint32_t)0x80000000)
#define BLE_PRND_INIT_POS    31
#define BLE_PRND_VAL_MASK    ((uint32_t)0x003FFFFF)
#define BLE_PRND_VAL_LSB     0
#define BLE_PRND_VAL_WIDTH   ((uint32_t)0x00000016)

#define BLE_PRND_INIT_RST    0x0
#define BLE_PRND_VAL_RST     0x30F0F0

__INLINE void ble_ral_peer_rnd_pack(uint8_t prndinit, uint32_t prndval)
{
    ASSERT_ERR((((uint32_t)prndinit << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)prndval << 0) & ~((uint32_t)0x003FFFFF)) == 0);
    REG_BLE_WR(BLE_RAL_PEER_RND_ADDR, ((uint32_t)prndinit << 31) | ((uint32_t)prndval << 0));
}

__INLINE void ble_ral_peer_rnd_unpack(uint8_t *prndinit, uint32_t *prndval)
{
    uint32_t localVal = REG_BLE_RD(BLE_RAL_PEER_RND_ADDR);

    *prndinit = (localVal & ((uint32_t)0x80000000)) >> 31;
    *prndval = (localVal & ((uint32_t)0x003FFFFF)) >> 0;
}

__INLINE uint8_t ble_ral_peer_rnd_prnd_init_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RAL_PEER_RND_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_ral_peer_rnd_prnd_init_setf(uint8_t prndinit)
{
    ASSERT_ERR((((uint32_t)prndinit << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_RAL_PEER_RND_ADDR, (REG_BLE_RD(BLE_RAL_PEER_RND_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)prndinit << 31));
}

__INLINE uint32_t ble_ral_peer_rnd_prnd_val_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_RAL_PEER_RND_ADDR);
    return ((localVal & ((uint32_t)0x003FFFFF)) >> 0);
}

__INLINE void ble_ral_peer_rnd_prnd_val_setf(uint32_t prndval)
{
    ASSERT_ERR((((uint32_t)prndval << 0) & ~((uint32_t)0x003FFFFF)) == 0);
    REG_BLE_WR(BLE_RAL_PEER_RND_ADDR, (REG_BLE_RD(BLE_RAL_PEER_RND_ADDR) & ~((uint32_t)0x003FFFFF)) | ((uint32_t)prndval << 0));
}

/**
 * @brief ISOCHANCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04           RETXACKEN0   0
 *     03             SYNCGEN0   0
 *     02           ISOCHANEN0   0
 *  01:00             ISOTYPE0   0x0
 * </pre>
 */
#define BLE_ISOCHANCNTL0_ADDR   0x40100180
#define BLE_ISOCHANCNTL0_OFFSET 0x00000180
#define BLE_ISOCHANCNTL0_INDEX  0x00000060
#define BLE_ISOCHANCNTL0_RESET  0x00000000

__INLINE uint32_t ble_isochancntl0_get(void)
{
    return REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR);
}

__INLINE void ble_isochancntl0_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCHANCNTL0_ADDR, value);
}

// field definitions
#define BLE_RETXACKEN0_BIT    ((uint32_t)0x00000010)
#define BLE_RETXACKEN0_POS    4
#define BLE_SYNCGEN0_BIT      ((uint32_t)0x00000008)
#define BLE_SYNCGEN0_POS      3
#define BLE_ISOCHANEN0_BIT    ((uint32_t)0x00000004)
#define BLE_ISOCHANEN0_POS    2
#define BLE_ISOTYPE0_MASK     ((uint32_t)0x00000003)
#define BLE_ISOTYPE0_LSB      0
#define BLE_ISOTYPE0_WIDTH    ((uint32_t)0x00000002)

#define BLE_RETXACKEN0_RST    0x0
#define BLE_SYNCGEN0_RST      0x0
#define BLE_ISOCHANEN0_RST    0x0
#define BLE_ISOTYPE0_RST      0x0

__INLINE void ble_isochancntl0_pack(uint8_t retxacken0, uint8_t syncgen0, uint8_t isochanen0, uint8_t isotype0)
{
    ASSERT_ERR((((uint32_t)retxacken0 << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)syncgen0 << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)isochanen0 << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)isotype0 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL0_ADDR, ((uint32_t)retxacken0 << 4) | ((uint32_t)syncgen0 << 3) | ((uint32_t)isochanen0 << 2) | ((uint32_t)isotype0 << 0));
}

__INLINE void ble_isochancntl0_unpack(uint8_t *retxacken0, uint8_t *syncgen0, uint8_t *isochanen0, uint8_t *isotype0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR);

    *retxacken0 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *syncgen0 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *isochanen0 = (localVal & ((uint32_t)0x00000004)) >> 2;
    *isotype0 = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t ble_isochancntl0_retxacken0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ble_isochancntl0_retxacken0_setf(uint8_t retxacken0)
{
    ASSERT_ERR((((uint32_t)retxacken0 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL0_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)retxacken0 << 4));
}

__INLINE uint8_t ble_isochancntl0_syncgen0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_isochancntl0_syncgen0_setf(uint8_t syncgen0)
{
    ASSERT_ERR((((uint32_t)syncgen0 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL0_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)syncgen0 << 3));
}

__INLINE uint8_t ble_isochancntl0_isochanen0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_isochancntl0_isochanen0_setf(uint8_t isochanen0)
{
    ASSERT_ERR((((uint32_t)isochanen0 << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL0_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)isochanen0 << 2));
}

__INLINE uint8_t ble_isochancntl0_isotype0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void ble_isochancntl0_isotype0_setf(uint8_t isotype0)
{
    ASSERT_ERR((((uint32_t)isotype0 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL0_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL0_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)isotype0 << 0));
}

/**
 * @brief ISOMUTECNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                 TOG0   0
 *     21           MUTE_SINK0   0
 *     20         MUTE_SOURCE0   0
 *  19:18              INVL0_1   0x2
 *  17:16              INVL0_0   0x2
 *  07:00        MUTE_PATTERN0   0x0
 * </pre>
 */
#define BLE_ISOMUTECNTL0_ADDR   0x40100184
#define BLE_ISOMUTECNTL0_OFFSET 0x00000184
#define BLE_ISOMUTECNTL0_INDEX  0x00000061
#define BLE_ISOMUTECNTL0_RESET  0x000A0000

__INLINE uint32_t ble_isomutecntl0_get(void)
{
    return REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);
}

__INLINE void ble_isomutecntl0_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOMUTECNTL0_ADDR, value);
}

// field definitions
#define BLE_TOG0_BIT             ((uint32_t)0x80000000)
#define BLE_TOG0_POS             31
#define BLE_MUTE_SINK0_BIT       ((uint32_t)0x00200000)
#define BLE_MUTE_SINK0_POS       21
#define BLE_MUTE_SOURCE0_BIT     ((uint32_t)0x00100000)
#define BLE_MUTE_SOURCE0_POS     20
#define BLE_INVL0_1_MASK         ((uint32_t)0x000C0000)
#define BLE_INVL0_1_LSB          18
#define BLE_INVL0_1_WIDTH        ((uint32_t)0x00000002)
#define BLE_INVL0_0_MASK         ((uint32_t)0x00030000)
#define BLE_INVL0_0_LSB          16
#define BLE_INVL0_0_WIDTH        ((uint32_t)0x00000002)
#define BLE_MUTE_PATTERN0_MASK   ((uint32_t)0x000000FF)
#define BLE_MUTE_PATTERN0_LSB    0
#define BLE_MUTE_PATTERN0_WIDTH  ((uint32_t)0x00000008)

#define BLE_TOG0_RST             0x0
#define BLE_MUTE_SINK0_RST       0x0
#define BLE_MUTE_SOURCE0_RST     0x0
#define BLE_INVL0_1_RST          0x2
#define BLE_INVL0_0_RST          0x2
#define BLE_MUTE_PATTERN0_RST    0x0

__INLINE void ble_isomutecntl0_pack(uint8_t mutesink0, uint8_t mutesource0, uint8_t invl01, uint8_t invl00, uint8_t mutepattern0)
{
    ASSERT_ERR((((uint32_t)mutesink0 << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)mutesource0 << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)invl01 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)invl00 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mutepattern0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL0_ADDR, ((uint32_t)mutesink0 << 21) | ((uint32_t)mutesource0 << 20) | ((uint32_t)invl01 << 18) | ((uint32_t)invl00 << 16) | ((uint32_t)mutepattern0 << 0));
}

__INLINE void ble_isomutecntl0_unpack(uint8_t *tog0, uint8_t *mutesink0, uint8_t *mutesource0, uint8_t *invl01, uint8_t *invl00, uint8_t *mutepattern0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);

    *tog0 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mutesink0 = (localVal & ((uint32_t)0x00200000)) >> 21;
    *mutesource0 = (localVal & ((uint32_t)0x00100000)) >> 20;
    *invl01 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *invl00 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mutepattern0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_isomutecntl0_tog0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t ble_isomutecntl0_mute_sink0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void ble_isomutecntl0_mute_sink0_setf(uint8_t mutesink0)
{
    ASSERT_ERR((((uint32_t)mutesink0 << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL0_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)mutesink0 << 21));
}

__INLINE uint8_t ble_isomutecntl0_mute_source0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void ble_isomutecntl0_mute_source0_setf(uint8_t mutesource0)
{
    ASSERT_ERR((((uint32_t)mutesource0 << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL0_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)mutesource0 << 20));
}

__INLINE uint8_t ble_isomutecntl0_invl0_1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void ble_isomutecntl0_invl0_1_setf(uint8_t invl01)
{
    ASSERT_ERR((((uint32_t)invl01 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL0_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)invl01 << 18));
}

__INLINE uint8_t ble_isomutecntl0_invl0_0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void ble_isomutecntl0_invl0_0_setf(uint8_t invl00)
{
    ASSERT_ERR((((uint32_t)invl00 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL0_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)invl00 << 16));
}

__INLINE uint8_t ble_isomutecntl0_mute_pattern0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_isomutecntl0_mute_pattern0_setf(uint8_t mutepattern0)
{
    ASSERT_ERR((((uint32_t)mutepattern0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL0_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL0_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)mutepattern0 << 0));
}

/**
 * @brief ISOCURRENTTXPTR0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           ISO0TXPTR1   0x0
 *  15:00           ISO0TXPTR0   0x0
 * </pre>
 */
#define BLE_ISOCURRENTTXPTR0_ADDR   0x40100188
#define BLE_ISOCURRENTTXPTR0_OFFSET 0x00000188
#define BLE_ISOCURRENTTXPTR0_INDEX  0x00000062
#define BLE_ISOCURRENTTXPTR0_RESET  0x00000000

__INLINE uint32_t ble_isocurrenttxptr0_get(void)
{
    return REG_BLE_RD(BLE_ISOCURRENTTXPTR0_ADDR);
}

__INLINE void ble_isocurrenttxptr0_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCURRENTTXPTR0_ADDR, value);
}

// field definitions
#define BLE_ISO0TXPTR1_MASK   ((uint32_t)0xFFFF0000)
#define BLE_ISO0TXPTR1_LSB    16
#define BLE_ISO0TXPTR1_WIDTH  ((uint32_t)0x00000010)
#define BLE_ISO0TXPTR0_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ISO0TXPTR0_LSB    0
#define BLE_ISO0TXPTR0_WIDTH  ((uint32_t)0x00000010)

#define BLE_ISO0TXPTR1_RST    0x0
#define BLE_ISO0TXPTR0_RST    0x0

__INLINE void ble_isocurrenttxptr0_pack(uint16_t iso0txptr1, uint16_t iso0txptr0)
{
    ASSERT_ERR((((uint32_t)iso0txptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso0txptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR0_ADDR, ((uint32_t)iso0txptr1 << 16) | ((uint32_t)iso0txptr0 << 0));
}

__INLINE void ble_isocurrenttxptr0_unpack(uint16_t *iso0txptr1, uint16_t *iso0txptr0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR0_ADDR);

    *iso0txptr1 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *iso0txptr0 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_isocurrenttxptr0_iso0txptr1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR0_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_isocurrenttxptr0_iso0txptr1_setf(uint16_t iso0txptr1)
{
    ASSERT_ERR((((uint32_t)iso0txptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR0_ADDR, (REG_BLE_RD(BLE_ISOCURRENTTXPTR0_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)iso0txptr1 << 16));
}

__INLINE uint16_t ble_isocurrenttxptr0_iso0txptr0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_isocurrenttxptr0_iso0txptr0_setf(uint16_t iso0txptr0)
{
    ASSERT_ERR((((uint32_t)iso0txptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR0_ADDR, (REG_BLE_RD(BLE_ISOCURRENTTXPTR0_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)iso0txptr0 << 0));
}

/**
 * @brief ISOCURRENTRXPTR0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           ISO0RXPTR1   0x0
 *  15:00           ISO0RXPTR0   0x0
 * </pre>
 */
#define BLE_ISOCURRENTRXPTR0_ADDR   0x4010018C
#define BLE_ISOCURRENTRXPTR0_OFFSET 0x0000018C
#define BLE_ISOCURRENTRXPTR0_INDEX  0x00000063
#define BLE_ISOCURRENTRXPTR0_RESET  0x00000000

__INLINE uint32_t ble_isocurrentrxptr0_get(void)
{
    return REG_BLE_RD(BLE_ISOCURRENTRXPTR0_ADDR);
}

__INLINE void ble_isocurrentrxptr0_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCURRENTRXPTR0_ADDR, value);
}

// field definitions
#define BLE_ISO0RXPTR1_MASK   ((uint32_t)0xFFFF0000)
#define BLE_ISO0RXPTR1_LSB    16
#define BLE_ISO0RXPTR1_WIDTH  ((uint32_t)0x00000010)
#define BLE_ISO0RXPTR0_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ISO0RXPTR0_LSB    0
#define BLE_ISO0RXPTR0_WIDTH  ((uint32_t)0x00000010)

#define BLE_ISO0RXPTR1_RST    0x0
#define BLE_ISO0RXPTR0_RST    0x0

__INLINE void ble_isocurrentrxptr0_pack(uint16_t iso0rxptr1, uint16_t iso0rxptr0)
{
    ASSERT_ERR((((uint32_t)iso0rxptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso0rxptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR0_ADDR, ((uint32_t)iso0rxptr1 << 16) | ((uint32_t)iso0rxptr0 << 0));
}

__INLINE void ble_isocurrentrxptr0_unpack(uint16_t *iso0rxptr1, uint16_t *iso0rxptr0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR0_ADDR);

    *iso0rxptr1 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *iso0rxptr0 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_isocurrentrxptr0_iso0rxptr1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR0_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_isocurrentrxptr0_iso0rxptr1_setf(uint16_t iso0rxptr1)
{
    ASSERT_ERR((((uint32_t)iso0rxptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR0_ADDR, (REG_BLE_RD(BLE_ISOCURRENTRXPTR0_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)iso0rxptr1 << 16));
}

__INLINE uint16_t ble_isocurrentrxptr0_iso0rxptr0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_isocurrentrxptr0_iso0rxptr0_setf(uint16_t iso0rxptr0)
{
    ASSERT_ERR((((uint32_t)iso0rxptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR0_ADDR, (REG_BLE_RD(BLE_ISOCURRENTRXPTR0_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)iso0rxptr0 << 0));
}

/**
 * @brief ISOTRCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16            ISO0RXLEN   0x0
 *  07:00            ISO0TXLEN   0x0
 * </pre>
 */
#define BLE_ISOTRCNTL0_ADDR   0x40100190
#define BLE_ISOTRCNTL0_OFFSET 0x00000190
#define BLE_ISOTRCNTL0_INDEX  0x00000064
#define BLE_ISOTRCNTL0_RESET  0x00000000

__INLINE uint32_t ble_isotrcntl0_get(void)
{
    return REG_BLE_RD(BLE_ISOTRCNTL0_ADDR);
}

__INLINE void ble_isotrcntl0_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOTRCNTL0_ADDR, value);
}

// field definitions
#define BLE_ISO0RXLEN_MASK   ((uint32_t)0x00FF0000)
#define BLE_ISO0RXLEN_LSB    16
#define BLE_ISO0RXLEN_WIDTH  ((uint32_t)0x00000008)
#define BLE_ISO0TXLEN_MASK   ((uint32_t)0x000000FF)
#define BLE_ISO0TXLEN_LSB    0
#define BLE_ISO0TXLEN_WIDTH  ((uint32_t)0x00000008)

#define BLE_ISO0RXLEN_RST    0x0
#define BLE_ISO0TXLEN_RST    0x0

__INLINE void ble_isotrcntl0_pack(uint8_t iso0rxlen, uint8_t iso0txlen)
{
    ASSERT_ERR((((uint32_t)iso0rxlen << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso0txlen << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL0_ADDR, ((uint32_t)iso0rxlen << 16) | ((uint32_t)iso0txlen << 0));
}

__INLINE void ble_isotrcntl0_unpack(uint8_t *iso0rxlen, uint8_t *iso0txlen)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL0_ADDR);

    *iso0rxlen = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *iso0txlen = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_isotrcntl0_iso0rxlen_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_isotrcntl0_iso0rxlen_setf(uint8_t iso0rxlen)
{
    ASSERT_ERR((((uint32_t)iso0rxlen << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL0_ADDR, (REG_BLE_RD(BLE_ISOTRCNTL0_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)iso0rxlen << 16));
}

__INLINE uint8_t ble_isotrcntl0_iso0txlen_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_isotrcntl0_iso0txlen_setf(uint8_t iso0txlen)
{
    ASSERT_ERR((((uint32_t)iso0txlen << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL0_ADDR, (REG_BLE_RD(BLE_ISOTRCNTL0_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)iso0txlen << 0));
}

/**
 * @brief ISOEVTCNTOFFSETL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00     EVT_CNT_OFFSETL0   0x0
 * </pre>
 */
#define BLE_ISOEVTCNTOFFSETL0_ADDR   0x40100194
#define BLE_ISOEVTCNTOFFSETL0_OFFSET 0x00000194
#define BLE_ISOEVTCNTOFFSETL0_INDEX  0x00000065
#define BLE_ISOEVTCNTOFFSETL0_RESET  0x00000000

__INLINE uint32_t ble_isoevtcntoffsetl0_get(void)
{
    return REG_BLE_RD(BLE_ISOEVTCNTOFFSETL0_ADDR);
}

__INLINE void ble_isoevtcntoffsetl0_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETL0_ADDR, value);
}

// field definitions
#define BLE_EVT_CNT_OFFSETL0_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_EVT_CNT_OFFSETL0_LSB    0
#define BLE_EVT_CNT_OFFSETL0_WIDTH  ((uint32_t)0x00000020)

#define BLE_EVT_CNT_OFFSETL0_RST    0x0

__INLINE uint32_t ble_isoevtcntoffsetl0_evt_cnt_offsetl0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOEVTCNTOFFSETL0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isoevtcntoffsetl0_evt_cnt_offsetl0_setf(uint32_t evtcntoffsetl0)
{
    ASSERT_ERR((((uint32_t)evtcntoffsetl0 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETL0_ADDR, (uint32_t)evtcntoffsetl0 << 0);
}

/**
 * @brief ISOEVTCNTOFFSETU0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:00     EVT_CNT_OFFSETU0   0x0
 * </pre>
 */
#define BLE_ISOEVTCNTOFFSETU0_ADDR   0x40100198
#define BLE_ISOEVTCNTOFFSETU0_OFFSET 0x00000198
#define BLE_ISOEVTCNTOFFSETU0_INDEX  0x00000066
#define BLE_ISOEVTCNTOFFSETU0_RESET  0x00000000

__INLINE uint32_t ble_isoevtcntoffsetu0_get(void)
{
    return REG_BLE_RD(BLE_ISOEVTCNTOFFSETU0_ADDR);
}

__INLINE void ble_isoevtcntoffsetu0_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETU0_ADDR, value);
}

// field definitions
#define BLE_EVT_CNT_OFFSETU0_MASK   ((uint32_t)0x0000007F)
#define BLE_EVT_CNT_OFFSETU0_LSB    0
#define BLE_EVT_CNT_OFFSETU0_WIDTH  ((uint32_t)0x00000007)

#define BLE_EVT_CNT_OFFSETU0_RST    0x0

__INLINE uint8_t ble_isoevtcntoffsetu0_evt_cnt_offsetu0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOEVTCNTOFFSETU0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000007F)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isoevtcntoffsetu0_evt_cnt_offsetu0_setf(uint8_t evtcntoffsetu0)
{
    ASSERT_ERR((((uint32_t)evtcntoffsetu0 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETU0_ADDR, (uint32_t)evtcntoffsetu0 << 0);
}

/**
 * @brief ISOCHANCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04           RETXACKEN1   0
 *     03             SYNCGEN1   0
 *     02           ISOCHANEN1   0
 *  01:00             ISOTYPE1   0x0
 * </pre>
 */
#define BLE_ISOCHANCNTL1_ADDR   0x401001A0
#define BLE_ISOCHANCNTL1_OFFSET 0x000001A0
#define BLE_ISOCHANCNTL1_INDEX  0x00000068
#define BLE_ISOCHANCNTL1_RESET  0x00000000

__INLINE uint32_t ble_isochancntl1_get(void)
{
    return REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR);
}

__INLINE void ble_isochancntl1_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCHANCNTL1_ADDR, value);
}

// field definitions
#define BLE_RETXACKEN1_BIT    ((uint32_t)0x00000010)
#define BLE_RETXACKEN1_POS    4
#define BLE_SYNCGEN1_BIT      ((uint32_t)0x00000008)
#define BLE_SYNCGEN1_POS      3
#define BLE_ISOCHANEN1_BIT    ((uint32_t)0x00000004)
#define BLE_ISOCHANEN1_POS    2
#define BLE_ISOTYPE1_MASK     ((uint32_t)0x00000003)
#define BLE_ISOTYPE1_LSB      0
#define BLE_ISOTYPE1_WIDTH    ((uint32_t)0x00000002)

#define BLE_RETXACKEN1_RST    0x0
#define BLE_SYNCGEN1_RST      0x0
#define BLE_ISOCHANEN1_RST    0x0
#define BLE_ISOTYPE1_RST      0x0

__INLINE void ble_isochancntl1_pack(uint8_t retxacken1, uint8_t syncgen1, uint8_t isochanen1, uint8_t isotype1)
{
    ASSERT_ERR((((uint32_t)retxacken1 << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)syncgen1 << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)isochanen1 << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)isotype1 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL1_ADDR, ((uint32_t)retxacken1 << 4) | ((uint32_t)syncgen1 << 3) | ((uint32_t)isochanen1 << 2) | ((uint32_t)isotype1 << 0));
}

__INLINE void ble_isochancntl1_unpack(uint8_t *retxacken1, uint8_t *syncgen1, uint8_t *isochanen1, uint8_t *isotype1)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR);

    *retxacken1 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *syncgen1 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *isochanen1 = (localVal & ((uint32_t)0x00000004)) >> 2;
    *isotype1 = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t ble_isochancntl1_retxacken1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ble_isochancntl1_retxacken1_setf(uint8_t retxacken1)
{
    ASSERT_ERR((((uint32_t)retxacken1 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL1_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)retxacken1 << 4));
}

__INLINE uint8_t ble_isochancntl1_syncgen1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_isochancntl1_syncgen1_setf(uint8_t syncgen1)
{
    ASSERT_ERR((((uint32_t)syncgen1 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL1_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)syncgen1 << 3));
}

__INLINE uint8_t ble_isochancntl1_isochanen1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_isochancntl1_isochanen1_setf(uint8_t isochanen1)
{
    ASSERT_ERR((((uint32_t)isochanen1 << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL1_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)isochanen1 << 2));
}

__INLINE uint8_t ble_isochancntl1_isotype1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void ble_isochancntl1_isotype1_setf(uint8_t isotype1)
{
    ASSERT_ERR((((uint32_t)isotype1 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL1_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL1_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)isotype1 << 0));
}

/**
 * @brief ISOMUTECNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                 TOG1   0
 *     21           MUTE_SINK1   0
 *     20         MUTE_SOURCE1   0
 *  19:18              INVL1_1   0x2
 *  17:16              INVL1_0   0x2
 *  07:00        MUTE_PATTERN1   0x0
 * </pre>
 */
#define BLE_ISOMUTECNTL1_ADDR   0x401001A4
#define BLE_ISOMUTECNTL1_OFFSET 0x000001A4
#define BLE_ISOMUTECNTL1_INDEX  0x00000069
#define BLE_ISOMUTECNTL1_RESET  0x000A0000

__INLINE uint32_t ble_isomutecntl1_get(void)
{
    return REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);
}

__INLINE void ble_isomutecntl1_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOMUTECNTL1_ADDR, value);
}

// field definitions
#define BLE_TOG1_BIT             ((uint32_t)0x80000000)
#define BLE_TOG1_POS             31
#define BLE_MUTE_SINK1_BIT       ((uint32_t)0x00200000)
#define BLE_MUTE_SINK1_POS       21
#define BLE_MUTE_SOURCE1_BIT     ((uint32_t)0x00100000)
#define BLE_MUTE_SOURCE1_POS     20
#define BLE_INVL1_1_MASK         ((uint32_t)0x000C0000)
#define BLE_INVL1_1_LSB          18
#define BLE_INVL1_1_WIDTH        ((uint32_t)0x00000002)
#define BLE_INVL1_0_MASK         ((uint32_t)0x00030000)
#define BLE_INVL1_0_LSB          16
#define BLE_INVL1_0_WIDTH        ((uint32_t)0x00000002)
#define BLE_MUTE_PATTERN1_MASK   ((uint32_t)0x000000FF)
#define BLE_MUTE_PATTERN1_LSB    0
#define BLE_MUTE_PATTERN1_WIDTH  ((uint32_t)0x00000008)

#define BLE_TOG1_RST             0x0
#define BLE_MUTE_SINK1_RST       0x0
#define BLE_MUTE_SOURCE1_RST     0x0
#define BLE_INVL1_1_RST          0x2
#define BLE_INVL1_0_RST          0x2
#define BLE_MUTE_PATTERN1_RST    0x0

__INLINE void ble_isomutecntl1_pack(uint8_t mutesink1, uint8_t mutesource1, uint8_t invl11, uint8_t invl10, uint8_t mutepattern1)
{
    ASSERT_ERR((((uint32_t)mutesink1 << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)mutesource1 << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)invl11 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)invl10 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mutepattern1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL1_ADDR, ((uint32_t)mutesink1 << 21) | ((uint32_t)mutesource1 << 20) | ((uint32_t)invl11 << 18) | ((uint32_t)invl10 << 16) | ((uint32_t)mutepattern1 << 0));
}

__INLINE void ble_isomutecntl1_unpack(uint8_t *tog1, uint8_t *mutesink1, uint8_t *mutesource1, uint8_t *invl11, uint8_t *invl10, uint8_t *mutepattern1)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);

    *tog1 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mutesink1 = (localVal & ((uint32_t)0x00200000)) >> 21;
    *mutesource1 = (localVal & ((uint32_t)0x00100000)) >> 20;
    *invl11 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *invl10 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mutepattern1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_isomutecntl1_tog1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t ble_isomutecntl1_mute_sink1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void ble_isomutecntl1_mute_sink1_setf(uint8_t mutesink1)
{
    ASSERT_ERR((((uint32_t)mutesink1 << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL1_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)mutesink1 << 21));
}

__INLINE uint8_t ble_isomutecntl1_mute_source1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void ble_isomutecntl1_mute_source1_setf(uint8_t mutesource1)
{
    ASSERT_ERR((((uint32_t)mutesource1 << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL1_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)mutesource1 << 20));
}

__INLINE uint8_t ble_isomutecntl1_invl1_1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void ble_isomutecntl1_invl1_1_setf(uint8_t invl11)
{
    ASSERT_ERR((((uint32_t)invl11 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL1_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)invl11 << 18));
}

__INLINE uint8_t ble_isomutecntl1_invl1_0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void ble_isomutecntl1_invl1_0_setf(uint8_t invl10)
{
    ASSERT_ERR((((uint32_t)invl10 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL1_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)invl10 << 16));
}

__INLINE uint8_t ble_isomutecntl1_mute_pattern1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_isomutecntl1_mute_pattern1_setf(uint8_t mutepattern1)
{
    ASSERT_ERR((((uint32_t)mutepattern1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL1_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL1_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)mutepattern1 << 0));
}

/**
 * @brief ISOCURRENTTXPTR1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           ISO1TXPTR1   0x0
 *  15:00           ISO1TXPTR0   0x0
 * </pre>
 */
#define BLE_ISOCURRENTTXPTR1_ADDR   0x401001A8
#define BLE_ISOCURRENTTXPTR1_OFFSET 0x000001A8
#define BLE_ISOCURRENTTXPTR1_INDEX  0x0000006A
#define BLE_ISOCURRENTTXPTR1_RESET  0x00000000

__INLINE uint32_t ble_isocurrenttxptr1_get(void)
{
    return REG_BLE_RD(BLE_ISOCURRENTTXPTR1_ADDR);
}

__INLINE void ble_isocurrenttxptr1_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCURRENTTXPTR1_ADDR, value);
}

// field definitions
#define BLE_ISO1TXPTR1_MASK   ((uint32_t)0xFFFF0000)
#define BLE_ISO1TXPTR1_LSB    16
#define BLE_ISO1TXPTR1_WIDTH  ((uint32_t)0x00000010)
#define BLE_ISO1TXPTR0_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ISO1TXPTR0_LSB    0
#define BLE_ISO1TXPTR0_WIDTH  ((uint32_t)0x00000010)

#define BLE_ISO1TXPTR1_RST    0x0
#define BLE_ISO1TXPTR0_RST    0x0

__INLINE void ble_isocurrenttxptr1_pack(uint16_t iso1txptr1, uint16_t iso1txptr0)
{
    ASSERT_ERR((((uint32_t)iso1txptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso1txptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR1_ADDR, ((uint32_t)iso1txptr1 << 16) | ((uint32_t)iso1txptr0 << 0));
}

__INLINE void ble_isocurrenttxptr1_unpack(uint16_t *iso1txptr1, uint16_t *iso1txptr0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR1_ADDR);

    *iso1txptr1 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *iso1txptr0 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_isocurrenttxptr1_iso1txptr1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR1_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_isocurrenttxptr1_iso1txptr1_setf(uint16_t iso1txptr1)
{
    ASSERT_ERR((((uint32_t)iso1txptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR1_ADDR, (REG_BLE_RD(BLE_ISOCURRENTTXPTR1_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)iso1txptr1 << 16));
}

__INLINE uint16_t ble_isocurrenttxptr1_iso1txptr0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR1_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_isocurrenttxptr1_iso1txptr0_setf(uint16_t iso1txptr0)
{
    ASSERT_ERR((((uint32_t)iso1txptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR1_ADDR, (REG_BLE_RD(BLE_ISOCURRENTTXPTR1_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)iso1txptr0 << 0));
}

/**
 * @brief ISOCURRENTRXPTR1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           ISO1RXPTR1   0x0
 *  15:00           ISO1RXPTR0   0x0
 * </pre>
 */
#define BLE_ISOCURRENTRXPTR1_ADDR   0x401001AC
#define BLE_ISOCURRENTRXPTR1_OFFSET 0x000001AC
#define BLE_ISOCURRENTRXPTR1_INDEX  0x0000006B
#define BLE_ISOCURRENTRXPTR1_RESET  0x00000000

__INLINE uint32_t ble_isocurrentrxptr1_get(void)
{
    return REG_BLE_RD(BLE_ISOCURRENTRXPTR1_ADDR);
}

__INLINE void ble_isocurrentrxptr1_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCURRENTRXPTR1_ADDR, value);
}

// field definitions
#define BLE_ISO1RXPTR1_MASK   ((uint32_t)0xFFFF0000)
#define BLE_ISO1RXPTR1_LSB    16
#define BLE_ISO1RXPTR1_WIDTH  ((uint32_t)0x00000010)
#define BLE_ISO1RXPTR0_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ISO1RXPTR0_LSB    0
#define BLE_ISO1RXPTR0_WIDTH  ((uint32_t)0x00000010)

#define BLE_ISO1RXPTR1_RST    0x0
#define BLE_ISO1RXPTR0_RST    0x0

__INLINE void ble_isocurrentrxptr1_pack(uint16_t iso1rxptr1, uint16_t iso1rxptr0)
{
    ASSERT_ERR((((uint32_t)iso1rxptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso1rxptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR1_ADDR, ((uint32_t)iso1rxptr1 << 16) | ((uint32_t)iso1rxptr0 << 0));
}

__INLINE void ble_isocurrentrxptr1_unpack(uint16_t *iso1rxptr1, uint16_t *iso1rxptr0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR1_ADDR);

    *iso1rxptr1 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *iso1rxptr0 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_isocurrentrxptr1_iso1rxptr1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR1_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_isocurrentrxptr1_iso1rxptr1_setf(uint16_t iso1rxptr1)
{
    ASSERT_ERR((((uint32_t)iso1rxptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR1_ADDR, (REG_BLE_RD(BLE_ISOCURRENTRXPTR1_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)iso1rxptr1 << 16));
}

__INLINE uint16_t ble_isocurrentrxptr1_iso1rxptr0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR1_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_isocurrentrxptr1_iso1rxptr0_setf(uint16_t iso1rxptr0)
{
    ASSERT_ERR((((uint32_t)iso1rxptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR1_ADDR, (REG_BLE_RD(BLE_ISOCURRENTRXPTR1_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)iso1rxptr0 << 0));
}

/**
 * @brief ISOTRCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16            ISO1RXLEN   0x0
 *  07:00            ISO1TXLEN   0x0
 * </pre>
 */
#define BLE_ISOTRCNTL1_ADDR   0x401001B0
#define BLE_ISOTRCNTL1_OFFSET 0x000001B0
#define BLE_ISOTRCNTL1_INDEX  0x0000006C
#define BLE_ISOTRCNTL1_RESET  0x00000000

__INLINE uint32_t ble_isotrcntl1_get(void)
{
    return REG_BLE_RD(BLE_ISOTRCNTL1_ADDR);
}

__INLINE void ble_isotrcntl1_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOTRCNTL1_ADDR, value);
}

// field definitions
#define BLE_ISO1RXLEN_MASK   ((uint32_t)0x00FF0000)
#define BLE_ISO1RXLEN_LSB    16
#define BLE_ISO1RXLEN_WIDTH  ((uint32_t)0x00000008)
#define BLE_ISO1TXLEN_MASK   ((uint32_t)0x000000FF)
#define BLE_ISO1TXLEN_LSB    0
#define BLE_ISO1TXLEN_WIDTH  ((uint32_t)0x00000008)

#define BLE_ISO1RXLEN_RST    0x0
#define BLE_ISO1TXLEN_RST    0x0

__INLINE void ble_isotrcntl1_pack(uint8_t iso1rxlen, uint8_t iso1txlen)
{
    ASSERT_ERR((((uint32_t)iso1rxlen << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso1txlen << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL1_ADDR, ((uint32_t)iso1rxlen << 16) | ((uint32_t)iso1txlen << 0));
}

__INLINE void ble_isotrcntl1_unpack(uint8_t *iso1rxlen, uint8_t *iso1txlen)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL1_ADDR);

    *iso1rxlen = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *iso1txlen = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_isotrcntl1_iso1rxlen_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_isotrcntl1_iso1rxlen_setf(uint8_t iso1rxlen)
{
    ASSERT_ERR((((uint32_t)iso1rxlen << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL1_ADDR, (REG_BLE_RD(BLE_ISOTRCNTL1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)iso1rxlen << 16));
}

__INLINE uint8_t ble_isotrcntl1_iso1txlen_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_isotrcntl1_iso1txlen_setf(uint8_t iso1txlen)
{
    ASSERT_ERR((((uint32_t)iso1txlen << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL1_ADDR, (REG_BLE_RD(BLE_ISOTRCNTL1_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)iso1txlen << 0));
}

/**
 * @brief ISOEVTCNTOFFSETL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00     EVT_CNT_OFFSETL1   0x0
 * </pre>
 */
#define BLE_ISOEVTCNTOFFSETL1_ADDR   0x401001B4
#define BLE_ISOEVTCNTOFFSETL1_OFFSET 0x000001B4
#define BLE_ISOEVTCNTOFFSETL1_INDEX  0x0000006D
#define BLE_ISOEVTCNTOFFSETL1_RESET  0x00000000

__INLINE uint32_t ble_isoevtcntoffsetl1_get(void)
{
    return REG_BLE_RD(BLE_ISOEVTCNTOFFSETL1_ADDR);
}

__INLINE void ble_isoevtcntoffsetl1_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETL1_ADDR, value);
}

// field definitions
#define BLE_EVT_CNT_OFFSETL1_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_EVT_CNT_OFFSETL1_LSB    0
#define BLE_EVT_CNT_OFFSETL1_WIDTH  ((uint32_t)0x00000020)

#define BLE_EVT_CNT_OFFSETL1_RST    0x0

__INLINE uint32_t ble_isoevtcntoffsetl1_evt_cnt_offsetl1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOEVTCNTOFFSETL1_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isoevtcntoffsetl1_evt_cnt_offsetl1_setf(uint32_t evtcntoffsetl1)
{
    ASSERT_ERR((((uint32_t)evtcntoffsetl1 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETL1_ADDR, (uint32_t)evtcntoffsetl1 << 0);
}

/**
 * @brief ISOEVTCNTOFFSETU1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:00     EVT_CNT_OFFSETU1   0x0
 * </pre>
 */
#define BLE_ISOEVTCNTOFFSETU1_ADDR   0x401001B8
#define BLE_ISOEVTCNTOFFSETU1_OFFSET 0x000001B8
#define BLE_ISOEVTCNTOFFSETU1_INDEX  0x0000006E
#define BLE_ISOEVTCNTOFFSETU1_RESET  0x00000000

__INLINE uint32_t ble_isoevtcntoffsetu1_get(void)
{
    return REG_BLE_RD(BLE_ISOEVTCNTOFFSETU1_ADDR);
}

__INLINE void ble_isoevtcntoffsetu1_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETU1_ADDR, value);
}

// field definitions
#define BLE_EVT_CNT_OFFSETU1_MASK   ((uint32_t)0x0000007F)
#define BLE_EVT_CNT_OFFSETU1_LSB    0
#define BLE_EVT_CNT_OFFSETU1_WIDTH  ((uint32_t)0x00000007)

#define BLE_EVT_CNT_OFFSETU1_RST    0x0

__INLINE uint8_t ble_isoevtcntoffsetu1_evt_cnt_offsetu1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOEVTCNTOFFSETU1_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000007F)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isoevtcntoffsetu1_evt_cnt_offsetu1_setf(uint8_t evtcntoffsetu1)
{
    ASSERT_ERR((((uint32_t)evtcntoffsetu1 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETU1_ADDR, (uint32_t)evtcntoffsetu1 << 0);
}

/**
 * @brief ISOCHANCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04           RETXACKEN2   0
 *     03             SYNCGEN2   0
 *     02           ISOCHANEN2   0
 *  01:00             ISOTYPE2   0x0
 * </pre>
 */
#define BLE_ISOCHANCNTL2_ADDR   0x401001C0
#define BLE_ISOCHANCNTL2_OFFSET 0x000001C0
#define BLE_ISOCHANCNTL2_INDEX  0x00000070
#define BLE_ISOCHANCNTL2_RESET  0x00000000

__INLINE uint32_t ble_isochancntl2_get(void)
{
    return REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR);
}

__INLINE void ble_isochancntl2_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCHANCNTL2_ADDR, value);
}

// field definitions
#define BLE_RETXACKEN2_BIT    ((uint32_t)0x00000010)
#define BLE_RETXACKEN2_POS    4
#define BLE_SYNCGEN2_BIT      ((uint32_t)0x00000008)
#define BLE_SYNCGEN2_POS      3
#define BLE_ISOCHANEN2_BIT    ((uint32_t)0x00000004)
#define BLE_ISOCHANEN2_POS    2
#define BLE_ISOTYPE2_MASK     ((uint32_t)0x00000003)
#define BLE_ISOTYPE2_LSB      0
#define BLE_ISOTYPE2_WIDTH    ((uint32_t)0x00000002)

#define BLE_RETXACKEN2_RST    0x0
#define BLE_SYNCGEN2_RST      0x0
#define BLE_ISOCHANEN2_RST    0x0
#define BLE_ISOTYPE2_RST      0x0

__INLINE void ble_isochancntl2_pack(uint8_t retxacken2, uint8_t syncgen2, uint8_t isochanen2, uint8_t isotype2)
{
    ASSERT_ERR((((uint32_t)retxacken2 << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)syncgen2 << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)isochanen2 << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)isotype2 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL2_ADDR, ((uint32_t)retxacken2 << 4) | ((uint32_t)syncgen2 << 3) | ((uint32_t)isochanen2 << 2) | ((uint32_t)isotype2 << 0));
}

__INLINE void ble_isochancntl2_unpack(uint8_t *retxacken2, uint8_t *syncgen2, uint8_t *isochanen2, uint8_t *isotype2)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR);

    *retxacken2 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *syncgen2 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *isochanen2 = (localVal & ((uint32_t)0x00000004)) >> 2;
    *isotype2 = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t ble_isochancntl2_retxacken2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ble_isochancntl2_retxacken2_setf(uint8_t retxacken2)
{
    ASSERT_ERR((((uint32_t)retxacken2 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL2_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)retxacken2 << 4));
}

__INLINE uint8_t ble_isochancntl2_syncgen2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ble_isochancntl2_syncgen2_setf(uint8_t syncgen2)
{
    ASSERT_ERR((((uint32_t)syncgen2 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL2_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)syncgen2 << 3));
}

__INLINE uint8_t ble_isochancntl2_isochanen2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_isochancntl2_isochanen2_setf(uint8_t isochanen2)
{
    ASSERT_ERR((((uint32_t)isochanen2 << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL2_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)isochanen2 << 2));
}

__INLINE uint8_t ble_isochancntl2_isotype2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void ble_isochancntl2_isotype2_setf(uint8_t isotype2)
{
    ASSERT_ERR((((uint32_t)isotype2 << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_BLE_WR(BLE_ISOCHANCNTL2_ADDR, (REG_BLE_RD(BLE_ISOCHANCNTL2_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)isotype2 << 0));
}

/**
 * @brief ISOMUTECNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31                 TOG2   0
 *     21           MUTE_SINK2   0
 *     20         MUTE_SOURCE2   0
 *  19:18              INVL2_1   0x2
 *  17:16              INVL2_0   0x2
 *  07:00        MUTE_PATTERN2   0x0
 * </pre>
 */
#define BLE_ISOMUTECNTL2_ADDR   0x401001C4
#define BLE_ISOMUTECNTL2_OFFSET 0x000001C4
#define BLE_ISOMUTECNTL2_INDEX  0x00000071
#define BLE_ISOMUTECNTL2_RESET  0x000A0000

__INLINE uint32_t ble_isomutecntl2_get(void)
{
    return REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);
}

__INLINE void ble_isomutecntl2_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOMUTECNTL2_ADDR, value);
}

// field definitions
#define BLE_TOG2_BIT             ((uint32_t)0x80000000)
#define BLE_TOG2_POS             31
#define BLE_MUTE_SINK2_BIT       ((uint32_t)0x00200000)
#define BLE_MUTE_SINK2_POS       21
#define BLE_MUTE_SOURCE2_BIT     ((uint32_t)0x00100000)
#define BLE_MUTE_SOURCE2_POS     20
#define BLE_INVL2_1_MASK         ((uint32_t)0x000C0000)
#define BLE_INVL2_1_LSB          18
#define BLE_INVL2_1_WIDTH        ((uint32_t)0x00000002)
#define BLE_INVL2_0_MASK         ((uint32_t)0x00030000)
#define BLE_INVL2_0_LSB          16
#define BLE_INVL2_0_WIDTH        ((uint32_t)0x00000002)
#define BLE_MUTE_PATTERN2_MASK   ((uint32_t)0x000000FF)
#define BLE_MUTE_PATTERN2_LSB    0
#define BLE_MUTE_PATTERN2_WIDTH  ((uint32_t)0x00000008)

#define BLE_TOG2_RST             0x0
#define BLE_MUTE_SINK2_RST       0x0
#define BLE_MUTE_SOURCE2_RST     0x0
#define BLE_INVL2_1_RST          0x2
#define BLE_INVL2_0_RST          0x2
#define BLE_MUTE_PATTERN2_RST    0x0

__INLINE void ble_isomutecntl2_pack(uint8_t mutesink2, uint8_t mutesource2, uint8_t invl21, uint8_t invl20, uint8_t mutepattern2)
{
    ASSERT_ERR((((uint32_t)mutesink2 << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)mutesource2 << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)invl21 << 18) & ~((uint32_t)0x000C0000)) == 0);
    ASSERT_ERR((((uint32_t)invl20 << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mutepattern2 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL2_ADDR, ((uint32_t)mutesink2 << 21) | ((uint32_t)mutesource2 << 20) | ((uint32_t)invl21 << 18) | ((uint32_t)invl20 << 16) | ((uint32_t)mutepattern2 << 0));
}

__INLINE void ble_isomutecntl2_unpack(uint8_t *tog2, uint8_t *mutesink2, uint8_t *mutesource2, uint8_t *invl21, uint8_t *invl20, uint8_t *mutepattern2)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);

    *tog2 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mutesink2 = (localVal & ((uint32_t)0x00200000)) >> 21;
    *mutesource2 = (localVal & ((uint32_t)0x00100000)) >> 20;
    *invl21 = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *invl20 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mutepattern2 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_isomutecntl2_tog2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t ble_isomutecntl2_mute_sink2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void ble_isomutecntl2_mute_sink2_setf(uint8_t mutesink2)
{
    ASSERT_ERR((((uint32_t)mutesink2 << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL2_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)mutesink2 << 21));
}

__INLINE uint8_t ble_isomutecntl2_mute_source2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void ble_isomutecntl2_mute_source2_setf(uint8_t mutesource2)
{
    ASSERT_ERR((((uint32_t)mutesource2 << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL2_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)mutesource2 << 20));
}

__INLINE uint8_t ble_isomutecntl2_invl2_1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void ble_isomutecntl2_invl2_1_setf(uint8_t invl21)
{
    ASSERT_ERR((((uint32_t)invl21 << 18) & ~((uint32_t)0x000C0000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL2_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)invl21 << 18));
}

__INLINE uint8_t ble_isomutecntl2_invl2_0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void ble_isomutecntl2_invl2_0_setf(uint8_t invl20)
{
    ASSERT_ERR((((uint32_t)invl20 << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL2_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)invl20 << 16));
}

__INLINE uint8_t ble_isomutecntl2_mute_pattern2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_isomutecntl2_mute_pattern2_setf(uint8_t mutepattern2)
{
    ASSERT_ERR((((uint32_t)mutepattern2 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOMUTECNTL2_ADDR, (REG_BLE_RD(BLE_ISOMUTECNTL2_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)mutepattern2 << 0));
}

/**
 * @brief ISOCURRENTTXPTR2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           ISO2TXPTR1   0x0
 *  15:00           ISO2TXPTR0   0x0
 * </pre>
 */
#define BLE_ISOCURRENTTXPTR2_ADDR   0x401001C8
#define BLE_ISOCURRENTTXPTR2_OFFSET 0x000001C8
#define BLE_ISOCURRENTTXPTR2_INDEX  0x00000072
#define BLE_ISOCURRENTTXPTR2_RESET  0x00000000

__INLINE uint32_t ble_isocurrenttxptr2_get(void)
{
    return REG_BLE_RD(BLE_ISOCURRENTTXPTR2_ADDR);
}

__INLINE void ble_isocurrenttxptr2_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCURRENTTXPTR2_ADDR, value);
}

// field definitions
#define BLE_ISO2TXPTR1_MASK   ((uint32_t)0xFFFF0000)
#define BLE_ISO2TXPTR1_LSB    16
#define BLE_ISO2TXPTR1_WIDTH  ((uint32_t)0x00000010)
#define BLE_ISO2TXPTR0_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ISO2TXPTR0_LSB    0
#define BLE_ISO2TXPTR0_WIDTH  ((uint32_t)0x00000010)

#define BLE_ISO2TXPTR1_RST    0x0
#define BLE_ISO2TXPTR0_RST    0x0

__INLINE void ble_isocurrenttxptr2_pack(uint16_t iso2txptr1, uint16_t iso2txptr0)
{
    ASSERT_ERR((((uint32_t)iso2txptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso2txptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR2_ADDR, ((uint32_t)iso2txptr1 << 16) | ((uint32_t)iso2txptr0 << 0));
}

__INLINE void ble_isocurrenttxptr2_unpack(uint16_t *iso2txptr1, uint16_t *iso2txptr0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR2_ADDR);

    *iso2txptr1 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *iso2txptr0 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_isocurrenttxptr2_iso2txptr1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR2_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_isocurrenttxptr2_iso2txptr1_setf(uint16_t iso2txptr1)
{
    ASSERT_ERR((((uint32_t)iso2txptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR2_ADDR, (REG_BLE_RD(BLE_ISOCURRENTTXPTR2_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)iso2txptr1 << 16));
}

__INLINE uint16_t ble_isocurrenttxptr2_iso2txptr0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTTXPTR2_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_isocurrenttxptr2_iso2txptr0_setf(uint16_t iso2txptr0)
{
    ASSERT_ERR((((uint32_t)iso2txptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTTXPTR2_ADDR, (REG_BLE_RD(BLE_ISOCURRENTTXPTR2_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)iso2txptr0 << 0));
}

/**
 * @brief ISOCURRENTRXPTR2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16           ISO2RXPTR1   0x0
 *  15:00           ISO2RXPTR0   0x0
 * </pre>
 */
#define BLE_ISOCURRENTRXPTR2_ADDR   0x401001CC
#define BLE_ISOCURRENTRXPTR2_OFFSET 0x000001CC
#define BLE_ISOCURRENTRXPTR2_INDEX  0x00000073
#define BLE_ISOCURRENTRXPTR2_RESET  0x00000000

__INLINE uint32_t ble_isocurrentrxptr2_get(void)
{
    return REG_BLE_RD(BLE_ISOCURRENTRXPTR2_ADDR);
}

__INLINE void ble_isocurrentrxptr2_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCURRENTRXPTR2_ADDR, value);
}

// field definitions
#define BLE_ISO2RXPTR1_MASK   ((uint32_t)0xFFFF0000)
#define BLE_ISO2RXPTR1_LSB    16
#define BLE_ISO2RXPTR1_WIDTH  ((uint32_t)0x00000010)
#define BLE_ISO2RXPTR0_MASK   ((uint32_t)0x0000FFFF)
#define BLE_ISO2RXPTR0_LSB    0
#define BLE_ISO2RXPTR0_WIDTH  ((uint32_t)0x00000010)

#define BLE_ISO2RXPTR1_RST    0x0
#define BLE_ISO2RXPTR0_RST    0x0

__INLINE void ble_isocurrentrxptr2_pack(uint16_t iso2rxptr1, uint16_t iso2rxptr0)
{
    ASSERT_ERR((((uint32_t)iso2rxptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso2rxptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR2_ADDR, ((uint32_t)iso2rxptr1 << 16) | ((uint32_t)iso2rxptr0 << 0));
}

__INLINE void ble_isocurrentrxptr2_unpack(uint16_t *iso2rxptr1, uint16_t *iso2rxptr0)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR2_ADDR);

    *iso2rxptr1 = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *iso2rxptr0 = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ble_isocurrentrxptr2_iso2rxptr1_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR2_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ble_isocurrentrxptr2_iso2rxptr1_setf(uint16_t iso2rxptr1)
{
    ASSERT_ERR((((uint32_t)iso2rxptr1 << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR2_ADDR, (REG_BLE_RD(BLE_ISOCURRENTRXPTR2_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)iso2rxptr1 << 16));
}

__INLINE uint16_t ble_isocurrentrxptr2_iso2rxptr0_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCURRENTRXPTR2_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ble_isocurrentrxptr2_iso2rxptr0_setf(uint16_t iso2rxptr0)
{
    ASSERT_ERR((((uint32_t)iso2rxptr0 << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BLE_WR(BLE_ISOCURRENTRXPTR2_ADDR, (REG_BLE_RD(BLE_ISOCURRENTRXPTR2_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)iso2rxptr0 << 0));
}

/**
 * @brief ISOTRCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16            ISO2RXLEN   0x0
 *  07:00            ISO2TXLEN   0x0
 * </pre>
 */
#define BLE_ISOTRCNTL2_ADDR   0x401001D0
#define BLE_ISOTRCNTL2_OFFSET 0x000001D0
#define BLE_ISOTRCNTL2_INDEX  0x00000074
#define BLE_ISOTRCNTL2_RESET  0x00000000

__INLINE uint32_t ble_isotrcntl2_get(void)
{
    return REG_BLE_RD(BLE_ISOTRCNTL2_ADDR);
}

__INLINE void ble_isotrcntl2_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOTRCNTL2_ADDR, value);
}

// field definitions
#define BLE_ISO2RXLEN_MASK   ((uint32_t)0x00FF0000)
#define BLE_ISO2RXLEN_LSB    16
#define BLE_ISO2RXLEN_WIDTH  ((uint32_t)0x00000008)
#define BLE_ISO2TXLEN_MASK   ((uint32_t)0x000000FF)
#define BLE_ISO2TXLEN_LSB    0
#define BLE_ISO2TXLEN_WIDTH  ((uint32_t)0x00000008)

#define BLE_ISO2RXLEN_RST    0x0
#define BLE_ISO2TXLEN_RST    0x0

__INLINE void ble_isotrcntl2_pack(uint8_t iso2rxlen, uint8_t iso2txlen)
{
    ASSERT_ERR((((uint32_t)iso2rxlen << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)iso2txlen << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL2_ADDR, ((uint32_t)iso2rxlen << 16) | ((uint32_t)iso2txlen << 0));
}

__INLINE void ble_isotrcntl2_unpack(uint8_t *iso2rxlen, uint8_t *iso2txlen)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL2_ADDR);

    *iso2rxlen = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *iso2txlen = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_isotrcntl2_iso2rxlen_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ble_isotrcntl2_iso2rxlen_setf(uint8_t iso2rxlen)
{
    ASSERT_ERR((((uint32_t)iso2rxlen << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL2_ADDR, (REG_BLE_RD(BLE_ISOTRCNTL2_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)iso2rxlen << 16));
}

__INLINE uint8_t ble_isotrcntl2_iso2txlen_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOTRCNTL2_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_isotrcntl2_iso2txlen_setf(uint8_t iso2txlen)
{
    ASSERT_ERR((((uint32_t)iso2txlen << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOTRCNTL2_ADDR, (REG_BLE_RD(BLE_ISOTRCNTL2_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)iso2txlen << 0));
}

/**
 * @brief ISOEVTCNTOFFSETL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00     EVT_CNT_OFFSETL2   0x0
 * </pre>
 */
#define BLE_ISOEVTCNTOFFSETL2_ADDR   0x401001D4
#define BLE_ISOEVTCNTOFFSETL2_OFFSET 0x000001D4
#define BLE_ISOEVTCNTOFFSETL2_INDEX  0x00000075
#define BLE_ISOEVTCNTOFFSETL2_RESET  0x00000000

__INLINE uint32_t ble_isoevtcntoffsetl2_get(void)
{
    return REG_BLE_RD(BLE_ISOEVTCNTOFFSETL2_ADDR);
}

__INLINE void ble_isoevtcntoffsetl2_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETL2_ADDR, value);
}

// field definitions
#define BLE_EVT_CNT_OFFSETL2_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_EVT_CNT_OFFSETL2_LSB    0
#define BLE_EVT_CNT_OFFSETL2_WIDTH  ((uint32_t)0x00000020)

#define BLE_EVT_CNT_OFFSETL2_RST    0x0

__INLINE uint32_t ble_isoevtcntoffsetl2_evt_cnt_offsetl2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOEVTCNTOFFSETL2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isoevtcntoffsetl2_evt_cnt_offsetl2_setf(uint32_t evtcntoffsetl2)
{
    ASSERT_ERR((((uint32_t)evtcntoffsetl2 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETL2_ADDR, (uint32_t)evtcntoffsetl2 << 0);
}

/**
 * @brief ISOEVTCNTOFFSETU2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:00     EVT_CNT_OFFSETU2   0x0
 * </pre>
 */
#define BLE_ISOEVTCNTOFFSETU2_ADDR   0x401001D8
#define BLE_ISOEVTCNTOFFSETU2_OFFSET 0x000001D8
#define BLE_ISOEVTCNTOFFSETU2_INDEX  0x00000076
#define BLE_ISOEVTCNTOFFSETU2_RESET  0x00000000

__INLINE uint32_t ble_isoevtcntoffsetu2_get(void)
{
    return REG_BLE_RD(BLE_ISOEVTCNTOFFSETU2_ADDR);
}

__INLINE void ble_isoevtcntoffsetu2_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETU2_ADDR, value);
}

// field definitions
#define BLE_EVT_CNT_OFFSETU2_MASK   ((uint32_t)0x0000007F)
#define BLE_EVT_CNT_OFFSETU2_LSB    0
#define BLE_EVT_CNT_OFFSETU2_WIDTH  ((uint32_t)0x00000007)

#define BLE_EVT_CNT_OFFSETU2_RST    0x0

__INLINE uint8_t ble_isoevtcntoffsetu2_evt_cnt_offsetu2_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOEVTCNTOFFSETU2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000007F)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isoevtcntoffsetu2_evt_cnt_offsetu2_setf(uint8_t evtcntoffsetu2)
{
    ASSERT_ERR((((uint32_t)evtcntoffsetu2 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_BLE_WR(BLE_ISOEVTCNTOFFSETU2_ADDR, (uint32_t)evtcntoffsetu2 << 0);
}


#endif // _REG_BLECORE_H_

