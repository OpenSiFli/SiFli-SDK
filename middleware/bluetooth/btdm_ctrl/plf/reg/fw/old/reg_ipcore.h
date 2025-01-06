#ifndef _REG_IPCORE_H_
#define _REG_IPCORE_H_

#include <stdint.h>
#include "_reg_ipcore.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_IPCORE_COUNT 61

#define REG_IPCORE_DECODING_MASK 0x000000FF

/**
 * @brief RWDMCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31      MASTER_SOFT_RST   0
 *     30    MASTER_TGSOFT_RST   0
 *     29         REG_SOFT_RST   0
 *     28   RADIOCNTL_SOFT_RST   0
 *     27            SWINT_REQ   0
 * </pre>
 */
#define IP_RWDMCNTL_ADDR   0x40100000
#define IP_RWDMCNTL_OFFSET 0x00000000
#define IP_RWDMCNTL_INDEX  0x00000000
#define IP_RWDMCNTL_RESET  0x00000000

__INLINE uint32_t ip_rwdmcntl_get(void)
{
    return REG_IP_RD(IP_RWDMCNTL_ADDR);
}

__INLINE void ip_rwdmcntl_set(uint32_t value)
{
    REG_IP_WR(IP_RWDMCNTL_ADDR, value);
}

// field definitions
#define IP_MASTER_SOFT_RST_BIT       ((uint32_t)0x80000000)
#define IP_MASTER_SOFT_RST_POS       31
#define IP_MASTER_TGSOFT_RST_BIT     ((uint32_t)0x40000000)
#define IP_MASTER_TGSOFT_RST_POS     30
#define IP_REG_SOFT_RST_BIT          ((uint32_t)0x20000000)
#define IP_REG_SOFT_RST_POS          29
#define IP_RADIOCNTL_SOFT_RST_BIT    ((uint32_t)0x10000000)
#define IP_RADIOCNTL_SOFT_RST_POS    28
#define IP_SWINT_REQ_BIT             ((uint32_t)0x08000000)
#define IP_SWINT_REQ_POS             27

#define IP_MASTER_SOFT_RST_RST       0x0
#define IP_MASTER_TGSOFT_RST_RST     0x0
#define IP_REG_SOFT_RST_RST          0x0
#define IP_RADIOCNTL_SOFT_RST_RST    0x0
#define IP_SWINT_REQ_RST             0x0

__INLINE void ip_rwdmcntl_pack(uint8_t mastersoftrst, uint8_t mastertgsoftrst, uint8_t regsoftrst, uint8_t radiocntlsoftrst, uint8_t swintreq)
{
    ASSERT_ERR((((uint32_t)mastersoftrst << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)mastertgsoftrst << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)regsoftrst << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)radiocntlsoftrst << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)swintreq << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_IP_WR(IP_RWDMCNTL_ADDR, ((uint32_t)mastersoftrst << 31) | ((uint32_t)mastertgsoftrst << 30) | ((uint32_t)regsoftrst << 29) | ((uint32_t)radiocntlsoftrst << 28) | ((uint32_t)swintreq << 27));
}

__INLINE void ip_rwdmcntl_unpack(uint8_t *mastersoftrst, uint8_t *mastertgsoftrst, uint8_t *regsoftrst, uint8_t *radiocntlsoftrst, uint8_t *swintreq)
{
    uint32_t localVal = REG_IP_RD(IP_RWDMCNTL_ADDR);

    *mastersoftrst = (localVal & ((uint32_t)0x80000000)) >> 31;
    *mastertgsoftrst = (localVal & ((uint32_t)0x40000000)) >> 30;
    *regsoftrst = (localVal & ((uint32_t)0x20000000)) >> 29;
    *radiocntlsoftrst = (localVal & ((uint32_t)0x10000000)) >> 28;
    *swintreq = (localVal & ((uint32_t)0x08000000)) >> 27;
}

__INLINE uint8_t ip_rwdmcntl_master_soft_rst_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RWDMCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_rwdmcntl_master_soft_rst_setf(uint8_t mastersoftrst)
{
    ASSERT_ERR((((uint32_t)mastersoftrst << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_RWDMCNTL_ADDR, (REG_IP_RD(IP_RWDMCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)mastersoftrst << 31));
}

__INLINE uint8_t ip_rwdmcntl_master_tgsoft_rst_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RWDMCNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ip_rwdmcntl_master_tgsoft_rst_setf(uint8_t mastertgsoftrst)
{
    ASSERT_ERR((((uint32_t)mastertgsoftrst << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_IP_WR(IP_RWDMCNTL_ADDR, (REG_IP_RD(IP_RWDMCNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)mastertgsoftrst << 30));
}

__INLINE uint8_t ip_rwdmcntl_reg_soft_rst_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RWDMCNTL_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void ip_rwdmcntl_reg_soft_rst_setf(uint8_t regsoftrst)
{
    ASSERT_ERR((((uint32_t)regsoftrst << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_IP_WR(IP_RWDMCNTL_ADDR, (REG_IP_RD(IP_RWDMCNTL_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)regsoftrst << 29));
}

__INLINE uint8_t ip_rwdmcntl_radiocntl_soft_rst_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RWDMCNTL_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void ip_rwdmcntl_radiocntl_soft_rst_setf(uint8_t radiocntlsoftrst)
{
    ASSERT_ERR((((uint32_t)radiocntlsoftrst << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_IP_WR(IP_RWDMCNTL_ADDR, (REG_IP_RD(IP_RWDMCNTL_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)radiocntlsoftrst << 28));
}

__INLINE uint8_t ip_rwdmcntl_swint_req_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RWDMCNTL_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void ip_rwdmcntl_swint_req_setf(uint8_t swintreq)
{
    ASSERT_ERR((((uint32_t)swintreq << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_IP_WR(IP_RWDMCNTL_ADDR, (REG_IP_RD(IP_RWDMCNTL_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)swintreq << 27));
}

/**
 * @brief VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24                  TYP   0x9
 *  23:16                  REL   0x0
 *  15:08                  UPG   0x9
 *  07:00                BUILD   0x0
 * </pre>
 */
#define IP_VERSION_ADDR   0x40100004
#define IP_VERSION_OFFSET 0x00000004
#define IP_VERSION_INDEX  0x00000001
#define IP_VERSION_RESET  0x09000900

__INLINE uint32_t ip_version_get(void)
{
    return REG_IP_RD(IP_VERSION_ADDR);
}

// field definitions
#define IP_TYP_MASK     ((uint32_t)0xFF000000)
#define IP_TYP_LSB      24
#define IP_TYP_WIDTH    ((uint32_t)0x00000008)
#define IP_REL_MASK     ((uint32_t)0x00FF0000)
#define IP_REL_LSB      16
#define IP_REL_WIDTH    ((uint32_t)0x00000008)
#define IP_UPG_MASK     ((uint32_t)0x0000FF00)
#define IP_UPG_LSB      8
#define IP_UPG_WIDTH    ((uint32_t)0x00000008)
#define IP_BUILD_MASK   ((uint32_t)0x000000FF)
#define IP_BUILD_LSB    0
#define IP_BUILD_WIDTH  ((uint32_t)0x00000008)

#define IP_TYP_RST      0x9
#define IP_REL_RST      0x0
#define IP_UPG_RST      0x9
#define IP_BUILD_RST    0x0

__INLINE void ip_version_unpack(uint8_t *typ, uint8_t *rel, uint8_t *upg, uint8_t *build)
{
    uint32_t localVal = REG_IP_RD(IP_VERSION_ADDR);

    *typ = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rel = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *upg = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *build = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ip_version_typ_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_VERSION_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t ip_version_rel_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t ip_version_upg_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t ip_version_build_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief INTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:28         CLKNINTSRMSK   0x0
 *  27:24         CLKNINTSRVAL   0x0
 *     12             SWINTMSK   0
 *     11      TIMESTAMPINTMSK   0
 *     10        FINETGTINTMSK   0
 *     09       GROSSTGTINTMSK   0
 *     08          ERRORINTMSK   0
 *     07          CRYPTINTMSK   0
 *     03            SLPINTMSK   1
 *     00           CLKNINTMSK   1
 * </pre>
 */
#define IP_INTCNTL_ADDR   0x4010000C
#define IP_INTCNTL_OFFSET 0x0000000C
#define IP_INTCNTL_INDEX  0x00000003
#define IP_INTCNTL_RESET  0x00000009

__INLINE uint32_t ip_intcntl_get(void)
{
    return REG_IP_RD(IP_INTCNTL_ADDR);
}

__INLINE void ip_intcntl_set(uint32_t value)
{
    REG_IP_WR(IP_INTCNTL_ADDR, value);
}

// field definitions
#define IP_CLKNINTSRMSK_MASK      ((uint32_t)0x70000000)
#define IP_CLKNINTSRMSK_LSB       28
#define IP_CLKNINTSRMSK_WIDTH     ((uint32_t)0x00000003)
#define IP_CLKNINTSRVAL_MASK      ((uint32_t)0x0F000000)
#define IP_CLKNINTSRVAL_LSB       24
#define IP_CLKNINTSRVAL_WIDTH     ((uint32_t)0x00000004)
#define IP_SWINTMSK_BIT           ((uint32_t)0x00001000)
#define IP_SWINTMSK_POS           12
#define IP_TIMESTAMPINTMSK_BIT    ((uint32_t)0x00000800)
#define IP_TIMESTAMPINTMSK_POS    11
#define IP_FINETGTINTMSK_BIT      ((uint32_t)0x00000400)
#define IP_FINETGTINTMSK_POS      10
#define IP_GROSSTGTINTMSK_BIT     ((uint32_t)0x00000200)
#define IP_GROSSTGTINTMSK_POS     9
#define IP_ERRORINTMSK_BIT        ((uint32_t)0x00000100)
#define IP_ERRORINTMSK_POS        8
#define IP_CRYPTINTMSK_BIT        ((uint32_t)0x00000080)
#define IP_CRYPTINTMSK_POS        7
#define IP_SLPINTMSK_BIT          ((uint32_t)0x00000008)
#define IP_SLPINTMSK_POS          3
#define IP_CLKNINTMSK_BIT         ((uint32_t)0x00000001)
#define IP_CLKNINTMSK_POS         0

#define IP_CLKNINTSRMSK_RST       0x0
#define IP_CLKNINTSRVAL_RST       0x0
#define IP_SWINTMSK_RST           0x0
#define IP_TIMESTAMPINTMSK_RST    0x0
#define IP_FINETGTINTMSK_RST      0x0
#define IP_GROSSTGTINTMSK_RST     0x0
#define IP_ERRORINTMSK_RST        0x0
#define IP_CRYPTINTMSK_RST        0x0
#define IP_SLPINTMSK_RST          0x1
#define IP_CLKNINTMSK_RST         0x1

__INLINE void ip_intcntl_pack(uint8_t clknintsrmsk, uint8_t clknintsrval, uint8_t swintmsk, uint8_t timestampintmsk, uint8_t finetgtintmsk, uint8_t grosstgtintmsk, uint8_t errorintmsk, uint8_t cryptintmsk, uint8_t slpintmsk, uint8_t clknintmsk)
{
    ASSERT_ERR((((uint32_t)clknintsrmsk << 28) & ~((uint32_t)0x70000000)) == 0);
    ASSERT_ERR((((uint32_t)clknintsrval << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)swintmsk << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)timestampintmsk << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)finetgtintmsk << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)grosstgtintmsk << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)errorintmsk << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)cryptintmsk << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)slpintmsk << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)clknintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, ((uint32_t)clknintsrmsk << 28) | ((uint32_t)clknintsrval << 24) | ((uint32_t)swintmsk << 12) | ((uint32_t)timestampintmsk << 11) | ((uint32_t)finetgtintmsk << 10) | ((uint32_t)grosstgtintmsk << 9) | ((uint32_t)errorintmsk << 8) | ((uint32_t)cryptintmsk << 7) | ((uint32_t)slpintmsk << 3) | ((uint32_t)clknintmsk << 0));
}

__INLINE void ip_intcntl_unpack(uint8_t *clknintsrmsk, uint8_t *clknintsrval, uint8_t *swintmsk, uint8_t *timestampintmsk, uint8_t *finetgtintmsk, uint8_t *grosstgtintmsk, uint8_t *errorintmsk, uint8_t *cryptintmsk, uint8_t *slpintmsk, uint8_t *clknintmsk)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);

    *clknintsrmsk = (localVal & ((uint32_t)0x70000000)) >> 28;
    *clknintsrval = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *swintmsk = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestampintmsk = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtintmsk = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtintmsk = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintmsk = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintmsk = (localVal & ((uint32_t)0x00000080)) >> 7;
    *slpintmsk = (localVal & ((uint32_t)0x00000008)) >> 3;
    *clknintmsk = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_intcntl_clknintsrmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x70000000)) >> 28);
}

__INLINE void ip_intcntl_clknintsrmsk_setf(uint8_t clknintsrmsk)
{
    ASSERT_ERR((((uint32_t)clknintsrmsk << 28) & ~((uint32_t)0x70000000)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x70000000)) | ((uint32_t)clknintsrmsk << 28));
}

__INLINE uint8_t ip_intcntl_clknintsrval_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void ip_intcntl_clknintsrval_setf(uint8_t clknintsrval)
{
    ASSERT_ERR((((uint32_t)clknintsrval << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)clknintsrval << 24));
}

__INLINE uint8_t ip_intcntl_swintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ip_intcntl_swintmsk_setf(uint8_t swintmsk)
{
    ASSERT_ERR((((uint32_t)swintmsk << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)swintmsk << 12));
}

__INLINE uint8_t ip_intcntl_timestampintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ip_intcntl_timestampintmsk_setf(uint8_t timestampintmsk)
{
    ASSERT_ERR((((uint32_t)timestampintmsk << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)timestampintmsk << 11));
}

__INLINE uint8_t ip_intcntl_finetgtintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ip_intcntl_finetgtintmsk_setf(uint8_t finetgtintmsk)
{
    ASSERT_ERR((((uint32_t)finetgtintmsk << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)finetgtintmsk << 10));
}

__INLINE uint8_t ip_intcntl_grosstgtintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ip_intcntl_grosstgtintmsk_setf(uint8_t grosstgtintmsk)
{
    ASSERT_ERR((((uint32_t)grosstgtintmsk << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)grosstgtintmsk << 9));
}

__INLINE uint8_t ip_intcntl_errorintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ip_intcntl_errorintmsk_setf(uint8_t errorintmsk)
{
    ASSERT_ERR((((uint32_t)errorintmsk << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)errorintmsk << 8));
}

__INLINE uint8_t ip_intcntl_cryptintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ip_intcntl_cryptintmsk_setf(uint8_t cryptintmsk)
{
    ASSERT_ERR((((uint32_t)cryptintmsk << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)cryptintmsk << 7));
}

__INLINE uint8_t ip_intcntl_slpintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ip_intcntl_slpintmsk_setf(uint8_t slpintmsk)
{
    ASSERT_ERR((((uint32_t)slpintmsk << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)slpintmsk << 3));
}

__INLINE uint8_t ip_intcntl_clknintmsk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_intcntl_clknintmsk_setf(uint8_t clknintmsk)
{
    ASSERT_ERR((((uint32_t)clknintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_INTCNTL_ADDR, (REG_IP_RD(IP_INTCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)clknintmsk << 0));
}

/**
 * @brief INTSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     12            SWINTSTAT   0
 *     11     TIMESTAMPINTSTAT   0
 *     10       FINETGTINTSTAT   0
 *     09      GROSSTGTINTSTAT   0
 *     08         ERRORINTSTAT   0
 *     07         CRYPTINTSTAT   0
 *     03           SLPINTSTAT   0
 *     00          CLKNINTSTAT   0
 * </pre>
 */
#define IP_INTSTAT_ADDR   0x40100010
#define IP_INTSTAT_OFFSET 0x00000010
#define IP_INTSTAT_INDEX  0x00000004
#define IP_INTSTAT_RESET  0x00000000

__INLINE uint32_t ip_intstat_get(void)
{
    return REG_IP_RD(IP_INTSTAT_ADDR);
}

// field definitions
#define IP_SWINTSTAT_BIT           ((uint32_t)0x00001000)
#define IP_SWINTSTAT_POS           12
#define IP_TIMESTAMPINTSTAT_BIT    ((uint32_t)0x00000800)
#define IP_TIMESTAMPINTSTAT_POS    11
#define IP_FINETGTINTSTAT_BIT      ((uint32_t)0x00000400)
#define IP_FINETGTINTSTAT_POS      10
#define IP_GROSSTGTINTSTAT_BIT     ((uint32_t)0x00000200)
#define IP_GROSSTGTINTSTAT_POS     9
#define IP_ERRORINTSTAT_BIT        ((uint32_t)0x00000100)
#define IP_ERRORINTSTAT_POS        8
#define IP_CRYPTINTSTAT_BIT        ((uint32_t)0x00000080)
#define IP_CRYPTINTSTAT_POS        7
#define IP_SLPINTSTAT_BIT          ((uint32_t)0x00000008)
#define IP_SLPINTSTAT_POS          3
#define IP_CLKNINTSTAT_BIT         ((uint32_t)0x00000001)
#define IP_CLKNINTSTAT_POS         0

#define IP_SWINTSTAT_RST           0x0
#define IP_TIMESTAMPINTSTAT_RST    0x0
#define IP_FINETGTINTSTAT_RST      0x0
#define IP_GROSSTGTINTSTAT_RST     0x0
#define IP_ERRORINTSTAT_RST        0x0
#define IP_CRYPTINTSTAT_RST        0x0
#define IP_SLPINTSTAT_RST          0x0
#define IP_CLKNINTSTAT_RST         0x0

__INLINE void ip_intstat_unpack(uint8_t *swintstat, uint8_t *timestampintstat, uint8_t *finetgtintstat, uint8_t *grosstgtintstat, uint8_t *errorintstat, uint8_t *cryptintstat, uint8_t *slpintstat, uint8_t *clknintstat)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);

    *swintstat = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestampintstat = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtintstat = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtintstat = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintstat = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintstat = (localVal & ((uint32_t)0x00000080)) >> 7;
    *slpintstat = (localVal & ((uint32_t)0x00000008)) >> 3;
    *clknintstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_intstat_swintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t ip_intstat_timestampintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t ip_intstat_finetgtintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t ip_intstat_grosstgtintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t ip_intstat_errorintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t ip_intstat_cryptintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t ip_intstat_slpintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t ip_intstat_clknintstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief INTRAWSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     12         SWINTRAWSTAT   0
 *     11   TIMESTAMPINTRAWSTAT   0
 *     10    FINETGTINTRAWSTAT   0
 *     09   GROSSTGTINTRAWSTAT   0
 *     08      ERRORINTRAWSTAT   0
 *     07      CRYPTINTRAWSTAT   0
 *     03        SLPINTRAWSTAT   0
 *     00       CLKNINTRAWSTAT   0
 * </pre>
 */
#define IP_INTRAWSTAT_ADDR   0x40100014
#define IP_INTRAWSTAT_OFFSET 0x00000014
#define IP_INTRAWSTAT_INDEX  0x00000005
#define IP_INTRAWSTAT_RESET  0x00000000

__INLINE uint32_t ip_intrawstat_get(void)
{
    return REG_IP_RD(IP_INTRAWSTAT_ADDR);
}

// field definitions
#define IP_SWINTRAWSTAT_BIT           ((uint32_t)0x00001000)
#define IP_SWINTRAWSTAT_POS           12
#define IP_TIMESTAMPINTRAWSTAT_BIT    ((uint32_t)0x00000800)
#define IP_TIMESTAMPINTRAWSTAT_POS    11
#define IP_FINETGTINTRAWSTAT_BIT      ((uint32_t)0x00000400)
#define IP_FINETGTINTRAWSTAT_POS      10
#define IP_GROSSTGTINTRAWSTAT_BIT     ((uint32_t)0x00000200)
#define IP_GROSSTGTINTRAWSTAT_POS     9
#define IP_ERRORINTRAWSTAT_BIT        ((uint32_t)0x00000100)
#define IP_ERRORINTRAWSTAT_POS        8
#define IP_CRYPTINTRAWSTAT_BIT        ((uint32_t)0x00000080)
#define IP_CRYPTINTRAWSTAT_POS        7
#define IP_SLPINTRAWSTAT_BIT          ((uint32_t)0x00000008)
#define IP_SLPINTRAWSTAT_POS          3
#define IP_CLKNINTRAWSTAT_BIT         ((uint32_t)0x00000001)
#define IP_CLKNINTRAWSTAT_POS         0

#define IP_SWINTRAWSTAT_RST           0x0
#define IP_TIMESTAMPINTRAWSTAT_RST    0x0
#define IP_FINETGTINTRAWSTAT_RST      0x0
#define IP_GROSSTGTINTRAWSTAT_RST     0x0
#define IP_ERRORINTRAWSTAT_RST        0x0
#define IP_CRYPTINTRAWSTAT_RST        0x0
#define IP_SLPINTRAWSTAT_RST          0x0
#define IP_CLKNINTRAWSTAT_RST         0x0

__INLINE void ip_intrawstat_unpack(uint8_t *swintrawstat, uint8_t *timestampintrawstat, uint8_t *finetgtintrawstat, uint8_t *grosstgtintrawstat, uint8_t *errorintrawstat, uint8_t *cryptintrawstat, uint8_t *slpintrawstat, uint8_t *clknintrawstat)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);

    *swintrawstat = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestampintrawstat = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtintrawstat = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtintrawstat = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintrawstat = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintrawstat = (localVal & ((uint32_t)0x00000080)) >> 7;
    *slpintrawstat = (localVal & ((uint32_t)0x00000008)) >> 3;
    *clknintrawstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_intrawstat_swintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t ip_intrawstat_timestampintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t ip_intrawstat_finetgtintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t ip_intrawstat_grosstgtintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t ip_intrawstat_errorintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t ip_intrawstat_cryptintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t ip_intrawstat_slpintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t ip_intrawstat_clknintrawstat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief INTACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     12             SWINTACK   0
 *     11      TIMESTAMPINTACK   0
 *     10        FINETGTINTACK   0
 *     09       GROSSTGTINTACK   0
 *     08          ERRORINTACK   0
 *     07          CRYPTINTACK   0
 *     03            SLPINTACK   0
 *     00           CLKNINTACK   0
 * </pre>
 */
#define IP_INTACK_ADDR   0x40100018
#define IP_INTACK_OFFSET 0x00000018
#define IP_INTACK_INDEX  0x00000006
#define IP_INTACK_RESET  0x00000000

__INLINE uint32_t ip_intack_get(void)
{
    return REG_IP_RD(IP_INTACK_ADDR);
}

__INLINE void ip_intack_clear(uint32_t value)
{
    REG_IP_WR(IP_INTACK_ADDR, value);
}

// field definitions
#define IP_SWINTACK_BIT           ((uint32_t)0x00001000)
#define IP_SWINTACK_POS           12
#define IP_TIMESTAMPINTACK_BIT    ((uint32_t)0x00000800)
#define IP_TIMESTAMPINTACK_POS    11
#define IP_FINETGTINTACK_BIT      ((uint32_t)0x00000400)
#define IP_FINETGTINTACK_POS      10
#define IP_GROSSTGTINTACK_BIT     ((uint32_t)0x00000200)
#define IP_GROSSTGTINTACK_POS     9
#define IP_ERRORINTACK_BIT        ((uint32_t)0x00000100)
#define IP_ERRORINTACK_POS        8
#define IP_CRYPTINTACK_BIT        ((uint32_t)0x00000080)
#define IP_CRYPTINTACK_POS        7
#define IP_SLPINTACK_BIT          ((uint32_t)0x00000008)
#define IP_SLPINTACK_POS          3
#define IP_CLKNINTACK_BIT         ((uint32_t)0x00000001)
#define IP_CLKNINTACK_POS         0

#define IP_SWINTACK_RST           0x0
#define IP_TIMESTAMPINTACK_RST    0x0
#define IP_FINETGTINTACK_RST      0x0
#define IP_GROSSTGTINTACK_RST     0x0
#define IP_ERRORINTACK_RST        0x0
#define IP_CRYPTINTACK_RST        0x0
#define IP_SLPINTACK_RST          0x0
#define IP_CLKNINTACK_RST         0x0

__INLINE void ip_intack_pack(uint8_t swintack, uint8_t timestampintack, uint8_t finetgtintack, uint8_t grosstgtintack, uint8_t errorintack, uint8_t cryptintack, uint8_t slpintack, uint8_t clknintack)
{
    ASSERT_ERR((((uint32_t)swintack << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)timestampintack << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)finetgtintack << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)grosstgtintack << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)errorintack << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)cryptintack << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)slpintack << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)clknintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, ((uint32_t)swintack << 12) | ((uint32_t)timestampintack << 11) | ((uint32_t)finetgtintack << 10) | ((uint32_t)grosstgtintack << 9) | ((uint32_t)errorintack << 8) | ((uint32_t)cryptintack << 7) | ((uint32_t)slpintack << 3) | ((uint32_t)clknintack << 0));
}

__INLINE void ip_intack_unpack(uint8_t *swintack, uint8_t *timestampintack, uint8_t *finetgtintack, uint8_t *grosstgtintack, uint8_t *errorintack, uint8_t *cryptintack, uint8_t *slpintack, uint8_t *clknintack)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);

    *swintack = (localVal & ((uint32_t)0x00001000)) >> 12;
    *timestampintack = (localVal & ((uint32_t)0x00000800)) >> 11;
    *finetgtintack = (localVal & ((uint32_t)0x00000400)) >> 10;
    *grosstgtintack = (localVal & ((uint32_t)0x00000200)) >> 9;
    *errorintack = (localVal & ((uint32_t)0x00000100)) >> 8;
    *cryptintack = (localVal & ((uint32_t)0x00000080)) >> 7;
    *slpintack = (localVal & ((uint32_t)0x00000008)) >> 3;
    *clknintack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_intack_swintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ip_intack_swintack_clearf(uint8_t swintack)
{
    ASSERT_ERR((((uint32_t)swintack << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)swintack << 12);
}

__INLINE uint8_t ip_intack_timestampintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ip_intack_timestampintack_clearf(uint8_t timestampintack)
{
    ASSERT_ERR((((uint32_t)timestampintack << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)timestampintack << 11);
}

__INLINE uint8_t ip_intack_finetgtintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ip_intack_finetgtintack_clearf(uint8_t finetgtintack)
{
    ASSERT_ERR((((uint32_t)finetgtintack << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)finetgtintack << 10);
}

__INLINE uint8_t ip_intack_grosstgtintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ip_intack_grosstgtintack_clearf(uint8_t grosstgtintack)
{
    ASSERT_ERR((((uint32_t)grosstgtintack << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)grosstgtintack << 9);
}

__INLINE uint8_t ip_intack_errorintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ip_intack_errorintack_clearf(uint8_t errorintack)
{
    ASSERT_ERR((((uint32_t)errorintack << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)errorintack << 8);
}

__INLINE uint8_t ip_intack_cryptintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ip_intack_cryptintack_clearf(uint8_t cryptintack)
{
    ASSERT_ERR((((uint32_t)cryptintack << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)cryptintack << 7);
}

__INLINE uint8_t ip_intack_slpintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ip_intack_slpintack_clearf(uint8_t slpintack)
{
    ASSERT_ERR((((uint32_t)slpintack << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)slpintack << 3);
}

__INLINE uint8_t ip_intack_clknintack_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_intack_clknintack_clearf(uint8_t clknintack)
{
    ASSERT_ERR((((uint32_t)clknintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_INTACK_ADDR, (uint32_t)clknintack << 0);
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
#define IP_SLOTCLK_ADDR   0x4010001C
#define IP_SLOTCLK_OFFSET 0x0000001C
#define IP_SLOTCLK_INDEX  0x00000007
#define IP_SLOTCLK_RESET  0x00000000

__INLINE uint32_t ip_slotclk_get(void)
{
    return REG_IP_RD(IP_SLOTCLK_ADDR);
}

__INLINE void ip_slotclk_set(uint32_t value)
{
    REG_IP_WR(IP_SLOTCLK_ADDR, value);
}

// field definitions
#define IP_SAMP_BIT        ((uint32_t)0x80000000)
#define IP_SAMP_POS        31
#define IP_CLKN_UPD_BIT    ((uint32_t)0x40000000)
#define IP_CLKN_UPD_POS    30
#define IP_SCLK_MASK       ((uint32_t)0x0FFFFFFF)
#define IP_SCLK_LSB        0
#define IP_SCLK_WIDTH      ((uint32_t)0x0000001C)

#define IP_SAMP_RST        0x0
#define IP_CLKN_UPD_RST    0x0
#define IP_SCLK_RST        0x0

__INLINE void ip_slotclk_pack(uint8_t samp, uint8_t clknupd, uint32_t sclk)
{
    ASSERT_ERR((((uint32_t)samp << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)clknupd << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)sclk << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_IP_WR(IP_SLOTCLK_ADDR, ((uint32_t)samp << 31) | ((uint32_t)clknupd << 30) | ((uint32_t)sclk << 0));
}

__INLINE void ip_slotclk_unpack(uint8_t *samp, uint8_t *clknupd, uint32_t *sclk)
{
    uint32_t localVal = REG_IP_RD(IP_SLOTCLK_ADDR);

    *samp = (localVal & ((uint32_t)0x80000000)) >> 31;
    *clknupd = (localVal & ((uint32_t)0x40000000)) >> 30;
    *sclk = (localVal & ((uint32_t)0x0FFFFFFF)) >> 0;
}

__INLINE uint8_t ip_slotclk_samp_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_slotclk_samp_setf(uint8_t samp)
{
    ASSERT_ERR((((uint32_t)samp << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_SLOTCLK_ADDR, (REG_IP_RD(IP_SLOTCLK_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)samp << 31));
}

__INLINE uint8_t ip_slotclk_clkn_upd_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ip_slotclk_clkn_upd_setf(uint8_t clknupd)
{
    ASSERT_ERR((((uint32_t)clknupd << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_IP_WR(IP_SLOTCLK_ADDR, (REG_IP_RD(IP_SLOTCLK_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)clknupd << 30));
}

__INLINE uint32_t ip_slotclk_sclk_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SLOTCLK_ADDR);
    return ((localVal & ((uint32_t)0x0FFFFFFF)) >> 0);
}

__INLINE void ip_slotclk_sclk_setf(uint32_t sclk)
{
    ASSERT_ERR((((uint32_t)sclk << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_IP_WR(IP_SLOTCLK_ADDR, (REG_IP_RD(IP_SLOTCLK_ADDR) & ~((uint32_t)0x0FFFFFFF)) | ((uint32_t)sclk << 0));
}

/**
 * @brief FINETIMECNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00              FINECNT   0x0
 * </pre>
 */
#define IP_FINETIMECNT_ADDR   0x40100020
#define IP_FINETIMECNT_OFFSET 0x00000020
#define IP_FINETIMECNT_INDEX  0x00000008
#define IP_FINETIMECNT_RESET  0x00000000

__INLINE uint32_t ip_finetimecnt_get(void)
{
    return REG_IP_RD(IP_FINETIMECNT_ADDR);
}

// field definitions
#define IP_FINECNT_MASK   ((uint32_t)0x000003FF)
#define IP_FINECNT_LSB    0
#define IP_FINECNT_WIDTH  ((uint32_t)0x0000000A)

#define IP_FINECNT_RST    0x0

__INLINE uint16_t ip_finetimecnt_finecnt_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_FINETIMECNT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
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
#define IP_DEEPSLCNTL_ADDR   0x40100030
#define IP_DEEPSLCNTL_OFFSET 0x00000030
#define IP_DEEPSLCNTL_INDEX  0x0000000C
#define IP_DEEPSLCNTL_RESET  0x00000000

__INLINE uint32_t ip_deepslcntl_get(void)
{
    return REG_IP_RD(IP_DEEPSLCNTL_ADDR);
}

__INLINE void ip_deepslcntl_set(uint32_t value)
{
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, value);
}

// field definitions
#define IP_EXTWKUPDSB_BIT            ((uint32_t)0x80000000)
#define IP_EXTWKUPDSB_POS            31
#define IP_DEEP_SLEEP_STAT_BIT       ((uint32_t)0x00008000)
#define IP_DEEP_SLEEP_STAT_POS       15
#define IP_SOFT_WAKEUP_REQ_BIT       ((uint32_t)0x00000010)
#define IP_SOFT_WAKEUP_REQ_POS       4
#define IP_DEEP_SLEEP_CORR_EN_BIT    ((uint32_t)0x00000008)
#define IP_DEEP_SLEEP_CORR_EN_POS    3
#define IP_DEEP_SLEEP_ON_BIT         ((uint32_t)0x00000004)
#define IP_DEEP_SLEEP_ON_POS         2
#define IP_RADIO_SLEEP_EN_BIT        ((uint32_t)0x00000002)
#define IP_RADIO_SLEEP_EN_POS        1
#define IP_OSC_SLEEP_EN_BIT          ((uint32_t)0x00000001)
#define IP_OSC_SLEEP_EN_POS          0

#define IP_EXTWKUPDSB_RST            0x0
#define IP_DEEP_SLEEP_STAT_RST       0x0
#define IP_SOFT_WAKEUP_REQ_RST       0x0
#define IP_DEEP_SLEEP_CORR_EN_RST    0x0
#define IP_DEEP_SLEEP_ON_RST         0x0
#define IP_RADIO_SLEEP_EN_RST        0x0
#define IP_OSC_SLEEP_EN_RST          0x0

__INLINE void ip_deepslcntl_pack(uint8_t extwkupdsb, uint8_t softwakeupreq, uint8_t deepsleepcorren, uint8_t deepsleepon, uint8_t radiosleepen, uint8_t oscsleepen)
{
    ASSERT_ERR((((uint32_t)extwkupdsb << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)softwakeupreq << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)deepsleepcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)deepsleepon << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)radiosleepen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)oscsleepen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, ((uint32_t)extwkupdsb << 31) | ((uint32_t)softwakeupreq << 4) | ((uint32_t)deepsleepcorren << 3) | ((uint32_t)deepsleepon << 2) | ((uint32_t)radiosleepen << 1) | ((uint32_t)oscsleepen << 0));
}

__INLINE void ip_deepslcntl_unpack(uint8_t *extwkupdsb, uint8_t *deepsleepstat, uint8_t *softwakeupreq, uint8_t *deepsleepcorren, uint8_t *deepsleepon, uint8_t *radiosleepen, uint8_t *oscsleepen)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);

    *extwkupdsb = (localVal & ((uint32_t)0x80000000)) >> 31;
    *deepsleepstat = (localVal & ((uint32_t)0x00008000)) >> 15;
    *softwakeupreq = (localVal & ((uint32_t)0x00000010)) >> 4;
    *deepsleepcorren = (localVal & ((uint32_t)0x00000008)) >> 3;
    *deepsleepon = (localVal & ((uint32_t)0x00000004)) >> 2;
    *radiosleepen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *oscsleepen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_deepslcntl_extwkupdsb_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_deepslcntl_extwkupdsb_setf(uint8_t extwkupdsb)
{
    ASSERT_ERR((((uint32_t)extwkupdsb << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, (REG_IP_RD(IP_DEEPSLCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)extwkupdsb << 31));
}

__INLINE uint8_t ip_deepslcntl_deep_sleep_stat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t ip_deepslcntl_soft_wakeup_req_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ip_deepslcntl_soft_wakeup_req_setf(uint8_t softwakeupreq)
{
    ASSERT_ERR((((uint32_t)softwakeupreq << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, (REG_IP_RD(IP_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)softwakeupreq << 4));
}

__INLINE uint8_t ip_deepslcntl_deep_sleep_corr_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ip_deepslcntl_deep_sleep_corr_en_setf(uint8_t deepsleepcorren)
{
    ASSERT_ERR((((uint32_t)deepsleepcorren << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, (REG_IP_RD(IP_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)deepsleepcorren << 3));
}

__INLINE uint8_t ip_deepslcntl_deep_sleep_on_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ip_deepslcntl_deep_sleep_on_setf(uint8_t deepsleepon)
{
    ASSERT_ERR((((uint32_t)deepsleepon << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, (REG_IP_RD(IP_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)deepsleepon << 2));
}

__INLINE uint8_t ip_deepslcntl_radio_sleep_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ip_deepslcntl_radio_sleep_en_setf(uint8_t radiosleepen)
{
    ASSERT_ERR((((uint32_t)radiosleepen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, (REG_IP_RD(IP_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)radiosleepen << 1));
}

__INLINE uint8_t ip_deepslcntl_osc_sleep_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_deepslcntl_osc_sleep_en_setf(uint8_t oscsleepen)
{
    ASSERT_ERR((((uint32_t)oscsleepen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_DEEPSLCNTL_ADDR, (REG_IP_RD(IP_DEEPSLCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)oscsleepen << 0));
}

/**
 * @brief DEEPSLWKUP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           DEEPSLTIME   0x0
 * </pre>
 */
#define IP_DEEPSLWKUP_ADDR   0x40100034
#define IP_DEEPSLWKUP_OFFSET 0x00000034
#define IP_DEEPSLWKUP_INDEX  0x0000000D
#define IP_DEEPSLWKUP_RESET  0x00000000

__INLINE uint32_t ip_deepslwkup_get(void)
{
    return REG_IP_RD(IP_DEEPSLWKUP_ADDR);
}

__INLINE void ip_deepslwkup_set(uint32_t value)
{
    REG_IP_WR(IP_DEEPSLWKUP_ADDR, value);
}

// field definitions
#define IP_DEEPSLTIME_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_DEEPSLTIME_LSB    0
#define IP_DEEPSLTIME_WIDTH  ((uint32_t)0x00000020)

#define IP_DEEPSLTIME_RST    0x0

__INLINE uint32_t ip_deepslwkup_deepsltime_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLWKUP_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_deepslwkup_deepsltime_setf(uint32_t deepsltime)
{
    ASSERT_ERR((((uint32_t)deepsltime << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_DEEPSLWKUP_ADDR, (uint32_t)deepsltime << 0);
}

/**
 * @brief DEEPSLSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            DEEPSLDUR   0x0
 * </pre>
 */
#define IP_DEEPSLSTAT_ADDR   0x40100038
#define IP_DEEPSLSTAT_OFFSET 0x00000038
#define IP_DEEPSLSTAT_INDEX  0x0000000E
#define IP_DEEPSLSTAT_RESET  0x00000000

__INLINE uint32_t ip_deepslstat_get(void)
{
    return REG_IP_RD(IP_DEEPSLSTAT_ADDR);
}

// field definitions
#define IP_DEEPSLDUR_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_DEEPSLDUR_LSB    0
#define IP_DEEPSLDUR_WIDTH  ((uint32_t)0x00000020)

#define IP_DEEPSLDUR_RST    0x0

__INLINE uint32_t ip_deepslstat_deepsldur_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEEPSLSTAT_ADDR);
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
#define IP_ENBPRESET_ADDR   0x4010003C
#define IP_ENBPRESET_OFFSET 0x0000003C
#define IP_ENBPRESET_INDEX  0x0000000F
#define IP_ENBPRESET_RESET  0x14028020

__INLINE uint32_t ip_enbpreset_get(void)
{
    return REG_IP_RD(IP_ENBPRESET_ADDR);
}

__INLINE void ip_enbpreset_set(uint32_t value)
{
    REG_IP_WR(IP_ENBPRESET_ADDR, value);
}

// field definitions
#define IP_TWEXT_MASK   ((uint32_t)0xFFE00000)
#define IP_TWEXT_LSB    21
#define IP_TWEXT_WIDTH  ((uint32_t)0x0000000B)
#define IP_TWOSC_MASK   ((uint32_t)0x001FFC00)
#define IP_TWOSC_LSB    10
#define IP_TWOSC_WIDTH  ((uint32_t)0x0000000B)
#define IP_TWRM_MASK    ((uint32_t)0x000003FF)
#define IP_TWRM_LSB     0
#define IP_TWRM_WIDTH   ((uint32_t)0x0000000A)

#define IP_TWEXT_RST    0xA0
#define IP_TWOSC_RST    0xA0
#define IP_TWRM_RST     0x20

__INLINE void ip_enbpreset_pack(uint16_t twext, uint16_t twosc, uint16_t twrm)
{
    ASSERT_ERR((((uint32_t)twext << 21) & ~((uint32_t)0xFFE00000)) == 0);
    ASSERT_ERR((((uint32_t)twosc << 10) & ~((uint32_t)0x001FFC00)) == 0);
    ASSERT_ERR((((uint32_t)twrm << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(IP_ENBPRESET_ADDR, ((uint32_t)twext << 21) | ((uint32_t)twosc << 10) | ((uint32_t)twrm << 0));
}

__INLINE void ip_enbpreset_unpack(uint16_t *twext, uint16_t *twosc, uint16_t *twrm)
{
    uint32_t localVal = REG_IP_RD(IP_ENBPRESET_ADDR);

    *twext = (localVal & ((uint32_t)0xFFE00000)) >> 21;
    *twosc = (localVal & ((uint32_t)0x001FFC00)) >> 10;
    *twrm = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t ip_enbpreset_twext_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0xFFE00000)) >> 21);
}

__INLINE void ip_enbpreset_twext_setf(uint16_t twext)
{
    ASSERT_ERR((((uint32_t)twext << 21) & ~((uint32_t)0xFFE00000)) == 0);
    REG_IP_WR(IP_ENBPRESET_ADDR, (REG_IP_RD(IP_ENBPRESET_ADDR) & ~((uint32_t)0xFFE00000)) | ((uint32_t)twext << 21));
}

__INLINE uint16_t ip_enbpreset_twosc_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0x001FFC00)) >> 10);
}

__INLINE void ip_enbpreset_twosc_setf(uint16_t twosc)
{
    ASSERT_ERR((((uint32_t)twosc << 10) & ~((uint32_t)0x001FFC00)) == 0);
    REG_IP_WR(IP_ENBPRESET_ADDR, (REG_IP_RD(IP_ENBPRESET_ADDR) & ~((uint32_t)0x001FFC00)) | ((uint32_t)twosc << 10));
}

__INLINE uint16_t ip_enbpreset_twrm_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ENBPRESET_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void ip_enbpreset_twrm_setf(uint16_t twrm)
{
    ASSERT_ERR((((uint32_t)twrm << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(IP_ENBPRESET_ADDR, (REG_IP_RD(IP_ENBPRESET_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)twrm << 0));
}

/**
 * @brief FINECNTCORR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00          FINECNTCORR   0x0
 * </pre>
 */
#define IP_FINECNTCORR_ADDR   0x40100040
#define IP_FINECNTCORR_OFFSET 0x00000040
#define IP_FINECNTCORR_INDEX  0x00000010
#define IP_FINECNTCORR_RESET  0x00000000

__INLINE uint32_t ip_finecntcorr_get(void)
{
    return REG_IP_RD(IP_FINECNTCORR_ADDR);
}

__INLINE void ip_finecntcorr_set(uint32_t value)
{
    REG_IP_WR(IP_FINECNTCORR_ADDR, value);
}

// field definitions
#define IP_FINECNTCORR_MASK   ((uint32_t)0x000003FF)
#define IP_FINECNTCORR_LSB    0
#define IP_FINECNTCORR_WIDTH  ((uint32_t)0x0000000A)

#define IP_FINECNTCORR_RST    0x0

__INLINE uint16_t ip_finecntcorr_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_FINECNTCORR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_finecntcorr_setf(uint16_t finecntcorr)
{
    ASSERT_ERR((((uint32_t)finecntcorr << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(IP_FINECNTCORR_ADDR, (uint32_t)finecntcorr << 0);
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
#define IP_CLKNCNTCORR_ADDR   0x40100044
#define IP_CLKNCNTCORR_OFFSET 0x00000044
#define IP_CLKNCNTCORR_INDEX  0x00000011
#define IP_CLKNCNTCORR_RESET  0x00000000

__INLINE uint32_t ip_clkncntcorr_get(void)
{
    return REG_IP_RD(IP_CLKNCNTCORR_ADDR);
}

__INLINE void ip_clkncntcorr_set(uint32_t value)
{
    REG_IP_WR(IP_CLKNCNTCORR_ADDR, value);
}

// field definitions
#define IP_ABS_DELTA_BIT      ((uint32_t)0x80000000)
#define IP_ABS_DELTA_POS      31
#define IP_CLKNCNTCORR_MASK   ((uint32_t)0x0FFFFFFF)
#define IP_CLKNCNTCORR_LSB    0
#define IP_CLKNCNTCORR_WIDTH  ((uint32_t)0x0000001C)

#define IP_ABS_DELTA_RST      0x0
#define IP_CLKNCNTCORR_RST    0x0

__INLINE void ip_clkncntcorr_pack(uint8_t absdelta, uint32_t clkncntcorr)
{
    ASSERT_ERR((((uint32_t)absdelta << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)clkncntcorr << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_IP_WR(IP_CLKNCNTCORR_ADDR, ((uint32_t)absdelta << 31) | ((uint32_t)clkncntcorr << 0));
}

__INLINE void ip_clkncntcorr_unpack(uint8_t *absdelta, uint32_t *clkncntcorr)
{
    uint32_t localVal = REG_IP_RD(IP_CLKNCNTCORR_ADDR);

    *absdelta = (localVal & ((uint32_t)0x80000000)) >> 31;
    *clkncntcorr = (localVal & ((uint32_t)0x0FFFFFFF)) >> 0;
}

__INLINE uint8_t ip_clkncntcorr_abs_delta_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_CLKNCNTCORR_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_clkncntcorr_abs_delta_setf(uint8_t absdelta)
{
    ASSERT_ERR((((uint32_t)absdelta << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_CLKNCNTCORR_ADDR, (REG_IP_RD(IP_CLKNCNTCORR_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)absdelta << 31));
}

__INLINE uint32_t ip_clkncntcorr_clkncntcorr_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_CLKNCNTCORR_ADDR);
    return ((localVal & ((uint32_t)0x0FFFFFFF)) >> 0);
}

__INLINE void ip_clkncntcorr_clkncntcorr_setf(uint32_t clkncntcorr)
{
    ASSERT_ERR((((uint32_t)clkncntcorr << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_IP_WR(IP_CLKNCNTCORR_ADDR, (REG_IP_RD(IP_CLKNCNTCORR_ADDR) & ~((uint32_t)0x0FFFFFFF)) | ((uint32_t)clkncntcorr << 0));
}

/**
 * @brief DIAGCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31             DIAG3_EN   0
 *  29:24                DIAG3   0x0
 *     23             DIAG2_EN   0
 *  21:16                DIAG2   0x0
 *     15             DIAG1_EN   0
 *  13:08                DIAG1   0x0
 *     07             DIAG0_EN   0
 *  05:00                DIAG0   0x0
 * </pre>
 */
#define IP_DIAGCNTL_ADDR   0x40100050
#define IP_DIAGCNTL_OFFSET 0x00000050
#define IP_DIAGCNTL_INDEX  0x00000014
#define IP_DIAGCNTL_RESET  0x00000000

__INLINE uint32_t ip_diagcntl_get(void)
{
    return REG_IP_RD(IP_DIAGCNTL_ADDR);
}

__INLINE void ip_diagcntl_set(uint32_t value)
{
    REG_IP_WR(IP_DIAGCNTL_ADDR, value);
}

// field definitions
#define IP_DIAG3_EN_BIT    ((uint32_t)0x80000000)
#define IP_DIAG3_EN_POS    31
#define IP_DIAG3_MASK      ((uint32_t)0x3F000000)
#define IP_DIAG3_LSB       24
#define IP_DIAG3_WIDTH     ((uint32_t)0x00000006)
#define IP_DIAG2_EN_BIT    ((uint32_t)0x00800000)
#define IP_DIAG2_EN_POS    23
#define IP_DIAG2_MASK      ((uint32_t)0x003F0000)
#define IP_DIAG2_LSB       16
#define IP_DIAG2_WIDTH     ((uint32_t)0x00000006)
#define IP_DIAG1_EN_BIT    ((uint32_t)0x00008000)
#define IP_DIAG1_EN_POS    15
#define IP_DIAG1_MASK      ((uint32_t)0x00003F00)
#define IP_DIAG1_LSB       8
#define IP_DIAG1_WIDTH     ((uint32_t)0x00000006)
#define IP_DIAG0_EN_BIT    ((uint32_t)0x00000080)
#define IP_DIAG0_EN_POS    7
#define IP_DIAG0_MASK      ((uint32_t)0x0000003F)
#define IP_DIAG0_LSB       0
#define IP_DIAG0_WIDTH     ((uint32_t)0x00000006)

#define IP_DIAG3_EN_RST    0x0
#define IP_DIAG3_RST       0x0
#define IP_DIAG2_EN_RST    0x0
#define IP_DIAG2_RST       0x0
#define IP_DIAG1_EN_RST    0x0
#define IP_DIAG1_RST       0x0
#define IP_DIAG0_EN_RST    0x0
#define IP_DIAG0_RST       0x0

__INLINE void ip_diagcntl_pack(uint8_t diag3en, uint8_t diag3, uint8_t diag2en, uint8_t diag2, uint8_t diag1en, uint8_t diag1, uint8_t diag0en, uint8_t diag0)
{
    ASSERT_ERR((((uint32_t)diag3en << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)diag3 << 24) & ~((uint32_t)0x3F000000)) == 0);
    ASSERT_ERR((((uint32_t)diag2en << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)diag2 << 16) & ~((uint32_t)0x003F0000)) == 0);
    ASSERT_ERR((((uint32_t)diag1en << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)diag1 << 8) & ~((uint32_t)0x00003F00)) == 0);
    ASSERT_ERR((((uint32_t)diag0en << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)diag0 << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, ((uint32_t)diag3en << 31) | ((uint32_t)diag3 << 24) | ((uint32_t)diag2en << 23) | ((uint32_t)diag2 << 16) | ((uint32_t)diag1en << 15) | ((uint32_t)diag1 << 8) | ((uint32_t)diag0en << 7) | ((uint32_t)diag0 << 0));
}

__INLINE void ip_diagcntl_unpack(uint8_t *diag3en, uint8_t *diag3, uint8_t *diag2en, uint8_t *diag2, uint8_t *diag1en, uint8_t *diag1, uint8_t *diag0en, uint8_t *diag0)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);

    *diag3en = (localVal & ((uint32_t)0x80000000)) >> 31;
    *diag3 = (localVal & ((uint32_t)0x3F000000)) >> 24;
    *diag2en = (localVal & ((uint32_t)0x00800000)) >> 23;
    *diag2 = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *diag1en = (localVal & ((uint32_t)0x00008000)) >> 15;
    *diag1 = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *diag0en = (localVal & ((uint32_t)0x00000080)) >> 7;
    *diag0 = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t ip_diagcntl_diag3_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_diagcntl_diag3_en_setf(uint8_t diag3en)
{
    ASSERT_ERR((((uint32_t)diag3en << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)diag3en << 31));
}

__INLINE uint8_t ip_diagcntl_diag3_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x3F000000)) >> 24);
}

__INLINE void ip_diagcntl_diag3_setf(uint8_t diag3)
{
    ASSERT_ERR((((uint32_t)diag3 << 24) & ~((uint32_t)0x3F000000)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x3F000000)) | ((uint32_t)diag3 << 24));
}

__INLINE uint8_t ip_diagcntl_diag2_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void ip_diagcntl_diag2_en_setf(uint8_t diag2en)
{
    ASSERT_ERR((((uint32_t)diag2en << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)diag2en << 23));
}

__INLINE uint8_t ip_diagcntl_diag2_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE void ip_diagcntl_diag2_setf(uint8_t diag2)
{
    ASSERT_ERR((((uint32_t)diag2 << 16) & ~((uint32_t)0x003F0000)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x003F0000)) | ((uint32_t)diag2 << 16));
}

__INLINE uint8_t ip_diagcntl_diag1_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ip_diagcntl_diag1_en_setf(uint8_t diag1en)
{
    ASSERT_ERR((((uint32_t)diag1en << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)diag1en << 15));
}

__INLINE uint8_t ip_diagcntl_diag1_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE void ip_diagcntl_diag1_setf(uint8_t diag1)
{
    ASSERT_ERR((((uint32_t)diag1 << 8) & ~((uint32_t)0x00003F00)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x00003F00)) | ((uint32_t)diag1 << 8));
}

__INLINE uint8_t ip_diagcntl_diag0_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ip_diagcntl_diag0_en_setf(uint8_t diag0en)
{
    ASSERT_ERR((((uint32_t)diag0en << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)diag0en << 7));
}

__INLINE uint8_t ip_diagcntl_diag0_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

__INLINE void ip_diagcntl_diag0_setf(uint8_t diag0)
{
    ASSERT_ERR((((uint32_t)diag0 << 0) & ~((uint32_t)0x0000003F)) == 0);
    REG_IP_WR(IP_DIAGCNTL_ADDR, (REG_IP_RD(IP_DIAGCNTL_ADDR) & ~((uint32_t)0x0000003F)) | ((uint32_t)diag0 << 0));
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
#define IP_DIAGSTAT_ADDR   0x40100054
#define IP_DIAGSTAT_OFFSET 0x00000054
#define IP_DIAGSTAT_INDEX  0x00000015
#define IP_DIAGSTAT_RESET  0x00000000

__INLINE uint32_t ip_diagstat_get(void)
{
    return REG_IP_RD(IP_DIAGSTAT_ADDR);
}

// field definitions
#define IP_DIAG3STAT_MASK   ((uint32_t)0xFF000000)
#define IP_DIAG3STAT_LSB    24
#define IP_DIAG3STAT_WIDTH  ((uint32_t)0x00000008)
#define IP_DIAG2STAT_MASK   ((uint32_t)0x00FF0000)
#define IP_DIAG2STAT_LSB    16
#define IP_DIAG2STAT_WIDTH  ((uint32_t)0x00000008)
#define IP_DIAG1STAT_MASK   ((uint32_t)0x0000FF00)
#define IP_DIAG1STAT_LSB    8
#define IP_DIAG1STAT_WIDTH  ((uint32_t)0x00000008)
#define IP_DIAG0STAT_MASK   ((uint32_t)0x000000FF)
#define IP_DIAG0STAT_LSB    0
#define IP_DIAG0STAT_WIDTH  ((uint32_t)0x00000008)

#define IP_DIAG3STAT_RST    0x0
#define IP_DIAG2STAT_RST    0x0
#define IP_DIAG1STAT_RST    0x0
#define IP_DIAG0STAT_RST    0x0

__INLINE void ip_diagstat_unpack(uint8_t *diag3stat, uint8_t *diag2stat, uint8_t *diag1stat, uint8_t *diag0stat)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGSTAT_ADDR);

    *diag3stat = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *diag2stat = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *diag1stat = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *diag0stat = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ip_diagstat_diag3stat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t ip_diagstat_diag2stat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t ip_diagstat_diag1stat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGSTAT_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t ip_diagstat_diag0stat_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DIAGSTAT_ADDR);
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
#define IP_DEBUGADDMAX_ADDR   0x40100058
#define IP_DEBUGADDMAX_OFFSET 0x00000058
#define IP_DEBUGADDMAX_INDEX  0x00000016
#define IP_DEBUGADDMAX_RESET  0x00000000

__INLINE uint32_t ip_debugaddmax_get(void)
{
    return REG_IP_RD(IP_DEBUGADDMAX_ADDR);
}

__INLINE void ip_debugaddmax_set(uint32_t value)
{
    REG_IP_WR(IP_DEBUGADDMAX_ADDR, value);
}

// field definitions
#define IP_REG_ADDMAX_MASK   ((uint32_t)0xFFFF0000)
#define IP_REG_ADDMAX_LSB    16
#define IP_REG_ADDMAX_WIDTH  ((uint32_t)0x00000010)
#define IP_EM_ADDMAX_MASK    ((uint32_t)0x0000FFFF)
#define IP_EM_ADDMAX_LSB     0
#define IP_EM_ADDMAX_WIDTH   ((uint32_t)0x00000010)

#define IP_REG_ADDMAX_RST    0x0
#define IP_EM_ADDMAX_RST     0x0

__INLINE void ip_debugaddmax_pack(uint16_t regaddmax, uint16_t emaddmax)
{
    ASSERT_ERR((((uint32_t)regaddmax << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)emaddmax << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_IP_WR(IP_DEBUGADDMAX_ADDR, ((uint32_t)regaddmax << 16) | ((uint32_t)emaddmax << 0));
}

__INLINE void ip_debugaddmax_unpack(uint16_t *regaddmax, uint16_t *emaddmax)
{
    uint32_t localVal = REG_IP_RD(IP_DEBUGADDMAX_ADDR);

    *regaddmax = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *emaddmax = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ip_debugaddmax_reg_addmax_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEBUGADDMAX_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ip_debugaddmax_reg_addmax_setf(uint16_t regaddmax)
{
    ASSERT_ERR((((uint32_t)regaddmax << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_IP_WR(IP_DEBUGADDMAX_ADDR, (REG_IP_RD(IP_DEBUGADDMAX_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)regaddmax << 16));
}

__INLINE uint16_t ip_debugaddmax_em_addmax_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEBUGADDMAX_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ip_debugaddmax_em_addmax_setf(uint16_t emaddmax)
{
    ASSERT_ERR((((uint32_t)emaddmax << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_IP_WR(IP_DEBUGADDMAX_ADDR, (REG_IP_RD(IP_DEBUGADDMAX_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)emaddmax << 0));
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
#define IP_DEBUGADDMIN_ADDR   0x4010005C
#define IP_DEBUGADDMIN_OFFSET 0x0000005C
#define IP_DEBUGADDMIN_INDEX  0x00000017
#define IP_DEBUGADDMIN_RESET  0x00000000

__INLINE uint32_t ip_debugaddmin_get(void)
{
    return REG_IP_RD(IP_DEBUGADDMIN_ADDR);
}

__INLINE void ip_debugaddmin_set(uint32_t value)
{
    REG_IP_WR(IP_DEBUGADDMIN_ADDR, value);
}

// field definitions
#define IP_REG_ADDMIN_MASK   ((uint32_t)0xFFFF0000)
#define IP_REG_ADDMIN_LSB    16
#define IP_REG_ADDMIN_WIDTH  ((uint32_t)0x00000010)
#define IP_EM_ADDMIN_MASK    ((uint32_t)0x0000FFFF)
#define IP_EM_ADDMIN_LSB     0
#define IP_EM_ADDMIN_WIDTH   ((uint32_t)0x00000010)

#define IP_REG_ADDMIN_RST    0x0
#define IP_EM_ADDMIN_RST     0x0

__INLINE void ip_debugaddmin_pack(uint16_t regaddmin, uint16_t emaddmin)
{
    ASSERT_ERR((((uint32_t)regaddmin << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)emaddmin << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_IP_WR(IP_DEBUGADDMIN_ADDR, ((uint32_t)regaddmin << 16) | ((uint32_t)emaddmin << 0));
}

__INLINE void ip_debugaddmin_unpack(uint16_t *regaddmin, uint16_t *emaddmin)
{
    uint32_t localVal = REG_IP_RD(IP_DEBUGADDMIN_ADDR);

    *regaddmin = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *emaddmin = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ip_debugaddmin_reg_addmin_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEBUGADDMIN_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ip_debugaddmin_reg_addmin_setf(uint16_t regaddmin)
{
    ASSERT_ERR((((uint32_t)regaddmin << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_IP_WR(IP_DEBUGADDMIN_ADDR, (REG_IP_RD(IP_DEBUGADDMIN_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)regaddmin << 16));
}

__INLINE uint16_t ip_debugaddmin_em_addmin_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_DEBUGADDMIN_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ip_debugaddmin_em_addmin_setf(uint16_t emaddmin)
{
    ASSERT_ERR((((uint32_t)emaddmin << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_IP_WR(IP_DEBUGADDMIN_ADDR, (REG_IP_RD(IP_DEBUGADDMIN_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)emaddmin << 0));
}

/**
 * @brief ERRORTYPESTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01    ARBITER_SCH_ERROR   0
 *     00    RADIO_EMACC_ERROR   0
 * </pre>
 */
#define IP_ERRORTYPESTAT_ADDR   0x40100060
#define IP_ERRORTYPESTAT_OFFSET 0x00000060
#define IP_ERRORTYPESTAT_INDEX  0x00000018
#define IP_ERRORTYPESTAT_RESET  0x00000000

__INLINE uint32_t ip_errortypestat_get(void)
{
    return REG_IP_RD(IP_ERRORTYPESTAT_ADDR);
}

// field definitions
#define IP_ARBITER_SCH_ERROR_BIT    ((uint32_t)0x00000002)
#define IP_ARBITER_SCH_ERROR_POS    1
#define IP_RADIO_EMACC_ERROR_BIT    ((uint32_t)0x00000001)
#define IP_RADIO_EMACC_ERROR_POS    0

#define IP_ARBITER_SCH_ERROR_RST    0x0
#define IP_RADIO_EMACC_ERROR_RST    0x0

__INLINE void ip_errortypestat_unpack(uint8_t *arbiterscherror, uint8_t *radioemaccerror)
{
    uint32_t localVal = REG_IP_RD(IP_ERRORTYPESTAT_ADDR);

    *arbiterscherror = (localVal & ((uint32_t)0x00000002)) >> 1;
    *radioemaccerror = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_errortypestat_arbiter_sch_error_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ERRORTYPESTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ip_errortypestat_radio_emacc_error_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_ERRORTYPESTAT_ADDR);
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
#define IP_SWPROFILING_ADDR   0x40100064
#define IP_SWPROFILING_OFFSET 0x00000064
#define IP_SWPROFILING_INDEX  0x00000019
#define IP_SWPROFILING_RESET  0x00000000

__INLINE uint32_t ip_swprofiling_get(void)
{
    return REG_IP_RD(IP_SWPROFILING_ADDR);
}

__INLINE void ip_swprofiling_set(uint32_t value)
{
    REG_IP_WR(IP_SWPROFILING_ADDR, value);
}

// field definitions
#define IP_SWPROF31_BIT    ((uint32_t)0x80000000)
#define IP_SWPROF31_POS    31
#define IP_SWPROF30_BIT    ((uint32_t)0x40000000)
#define IP_SWPROF30_POS    30
#define IP_SWPROF29_BIT    ((uint32_t)0x20000000)
#define IP_SWPROF29_POS    29
#define IP_SWPROF28_BIT    ((uint32_t)0x10000000)
#define IP_SWPROF28_POS    28
#define IP_SWPROF27_BIT    ((uint32_t)0x08000000)
#define IP_SWPROF27_POS    27
#define IP_SWPROF26_BIT    ((uint32_t)0x04000000)
#define IP_SWPROF26_POS    26
#define IP_SWPROF25_BIT    ((uint32_t)0x02000000)
#define IP_SWPROF25_POS    25
#define IP_SWPROF24_BIT    ((uint32_t)0x01000000)
#define IP_SWPROF24_POS    24
#define IP_SWPROF23_BIT    ((uint32_t)0x00800000)
#define IP_SWPROF23_POS    23
#define IP_SWPROF22_BIT    ((uint32_t)0x00400000)
#define IP_SWPROF22_POS    22
#define IP_SWPROF21_BIT    ((uint32_t)0x00200000)
#define IP_SWPROF21_POS    21
#define IP_SWPROF20_BIT    ((uint32_t)0x00100000)
#define IP_SWPROF20_POS    20
#define IP_SWPROF19_BIT    ((uint32_t)0x00080000)
#define IP_SWPROF19_POS    19
#define IP_SWPROF18_BIT    ((uint32_t)0x00040000)
#define IP_SWPROF18_POS    18
#define IP_SWPROF17_BIT    ((uint32_t)0x00020000)
#define IP_SWPROF17_POS    17
#define IP_SWPROF16_BIT    ((uint32_t)0x00010000)
#define IP_SWPROF16_POS    16
#define IP_SWPROF15_BIT    ((uint32_t)0x00008000)
#define IP_SWPROF15_POS    15
#define IP_SWPROF14_BIT    ((uint32_t)0x00004000)
#define IP_SWPROF14_POS    14
#define IP_SWPROF13_BIT    ((uint32_t)0x00002000)
#define IP_SWPROF13_POS    13
#define IP_SWPROF12_BIT    ((uint32_t)0x00001000)
#define IP_SWPROF12_POS    12
#define IP_SWPROF11_BIT    ((uint32_t)0x00000800)
#define IP_SWPROF11_POS    11
#define IP_SWPROF10_BIT    ((uint32_t)0x00000400)
#define IP_SWPROF10_POS    10
#define IP_SWPROF9_BIT     ((uint32_t)0x00000200)
#define IP_SWPROF9_POS     9
#define IP_SWPROF8_BIT     ((uint32_t)0x00000100)
#define IP_SWPROF8_POS     8
#define IP_SWPROF7_BIT     ((uint32_t)0x00000080)
#define IP_SWPROF7_POS     7
#define IP_SWPROF6_BIT     ((uint32_t)0x00000040)
#define IP_SWPROF6_POS     6
#define IP_SWPROF5_BIT     ((uint32_t)0x00000020)
#define IP_SWPROF5_POS     5
#define IP_SWPROF4_BIT     ((uint32_t)0x00000010)
#define IP_SWPROF4_POS     4
#define IP_SWPROF3_BIT     ((uint32_t)0x00000008)
#define IP_SWPROF3_POS     3
#define IP_SWPROF2_BIT     ((uint32_t)0x00000004)
#define IP_SWPROF2_POS     2
#define IP_SWPROF1_BIT     ((uint32_t)0x00000002)
#define IP_SWPROF1_POS     1
#define IP_SWPROF0_BIT     ((uint32_t)0x00000001)
#define IP_SWPROF0_POS     0

#define IP_SWPROF31_RST    0x0
#define IP_SWPROF30_RST    0x0
#define IP_SWPROF29_RST    0x0
#define IP_SWPROF28_RST    0x0
#define IP_SWPROF27_RST    0x0
#define IP_SWPROF26_RST    0x0
#define IP_SWPROF25_RST    0x0
#define IP_SWPROF24_RST    0x0
#define IP_SWPROF23_RST    0x0
#define IP_SWPROF22_RST    0x0
#define IP_SWPROF21_RST    0x0
#define IP_SWPROF20_RST    0x0
#define IP_SWPROF19_RST    0x0
#define IP_SWPROF18_RST    0x0
#define IP_SWPROF17_RST    0x0
#define IP_SWPROF16_RST    0x0
#define IP_SWPROF15_RST    0x0
#define IP_SWPROF14_RST    0x0
#define IP_SWPROF13_RST    0x0
#define IP_SWPROF12_RST    0x0
#define IP_SWPROF11_RST    0x0
#define IP_SWPROF10_RST    0x0
#define IP_SWPROF9_RST     0x0
#define IP_SWPROF8_RST     0x0
#define IP_SWPROF7_RST     0x0
#define IP_SWPROF6_RST     0x0
#define IP_SWPROF5_RST     0x0
#define IP_SWPROF4_RST     0x0
#define IP_SWPROF3_RST     0x0
#define IP_SWPROF2_RST     0x0
#define IP_SWPROF1_RST     0x0
#define IP_SWPROF0_RST     0x0

__INLINE void ip_swprofiling_pack(uint8_t swprof31, uint8_t swprof30, uint8_t swprof29, uint8_t swprof28, uint8_t swprof27, uint8_t swprof26, uint8_t swprof25, uint8_t swprof24, uint8_t swprof23, uint8_t swprof22, uint8_t swprof21, uint8_t swprof20, uint8_t swprof19, uint8_t swprof18, uint8_t swprof17, uint8_t swprof16, uint8_t swprof15, uint8_t swprof14, uint8_t swprof13, uint8_t swprof12, uint8_t swprof11, uint8_t swprof10, uint8_t swprof9, uint8_t swprof8, uint8_t swprof7, uint8_t swprof6, uint8_t swprof5, uint8_t swprof4, uint8_t swprof3, uint8_t swprof2, uint8_t swprof1, uint8_t swprof0)
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
    REG_IP_WR(IP_SWPROFILING_ADDR, ((uint32_t)swprof31 << 31) | ((uint32_t)swprof30 << 30) | ((uint32_t)swprof29 << 29) | ((uint32_t)swprof28 << 28) | ((uint32_t)swprof27 << 27) | ((uint32_t)swprof26 << 26) | ((uint32_t)swprof25 << 25) | ((uint32_t)swprof24 << 24) | ((uint32_t)swprof23 << 23) | ((uint32_t)swprof22 << 22) | ((uint32_t)swprof21 << 21) | ((uint32_t)swprof20 << 20) | ((uint32_t)swprof19 << 19) | ((uint32_t)swprof18 << 18) | ((uint32_t)swprof17 << 17) | ((uint32_t)swprof16 << 16) | ((uint32_t)swprof15 << 15) | ((uint32_t)swprof14 << 14) | ((uint32_t)swprof13 << 13) | ((uint32_t)swprof12 << 12) | ((uint32_t)swprof11 << 11) | ((uint32_t)swprof10 << 10) | ((uint32_t)swprof9 << 9) | ((uint32_t)swprof8 << 8) | ((uint32_t)swprof7 << 7) | ((uint32_t)swprof6 << 6) | ((uint32_t)swprof5 << 5) | ((uint32_t)swprof4 << 4) | ((uint32_t)swprof3 << 3) | ((uint32_t)swprof2 << 2) | ((uint32_t)swprof1 << 1) | ((uint32_t)swprof0 << 0));
}

__INLINE void ip_swprofiling_unpack(uint8_t *swprof31, uint8_t *swprof30, uint8_t *swprof29, uint8_t *swprof28, uint8_t *swprof27, uint8_t *swprof26, uint8_t *swprof25, uint8_t *swprof24, uint8_t *swprof23, uint8_t *swprof22, uint8_t *swprof21, uint8_t *swprof20, uint8_t *swprof19, uint8_t *swprof18, uint8_t *swprof17, uint8_t *swprof16, uint8_t *swprof15, uint8_t *swprof14, uint8_t *swprof13, uint8_t *swprof12, uint8_t *swprof11, uint8_t *swprof10, uint8_t *swprof9, uint8_t *swprof8, uint8_t *swprof7, uint8_t *swprof6, uint8_t *swprof5, uint8_t *swprof4, uint8_t *swprof3, uint8_t *swprof2, uint8_t *swprof1, uint8_t *swprof0)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);

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

__INLINE uint8_t ip_swprofiling_swprof31_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_swprofiling_swprof31_setf(uint8_t swprof31)
{
    ASSERT_ERR((((uint32_t)swprof31 << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)swprof31 << 31));
}

__INLINE uint8_t ip_swprofiling_swprof30_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ip_swprofiling_swprof30_setf(uint8_t swprof30)
{
    ASSERT_ERR((((uint32_t)swprof30 << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)swprof30 << 30));
}

__INLINE uint8_t ip_swprofiling_swprof29_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void ip_swprofiling_swprof29_setf(uint8_t swprof29)
{
    ASSERT_ERR((((uint32_t)swprof29 << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)swprof29 << 29));
}

__INLINE uint8_t ip_swprofiling_swprof28_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void ip_swprofiling_swprof28_setf(uint8_t swprof28)
{
    ASSERT_ERR((((uint32_t)swprof28 << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)swprof28 << 28));
}

__INLINE uint8_t ip_swprofiling_swprof27_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void ip_swprofiling_swprof27_setf(uint8_t swprof27)
{
    ASSERT_ERR((((uint32_t)swprof27 << 27) & ~((uint32_t)0x08000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)swprof27 << 27));
}

__INLINE uint8_t ip_swprofiling_swprof26_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void ip_swprofiling_swprof26_setf(uint8_t swprof26)
{
    ASSERT_ERR((((uint32_t)swprof26 << 26) & ~((uint32_t)0x04000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)swprof26 << 26));
}

__INLINE uint8_t ip_swprofiling_swprof25_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void ip_swprofiling_swprof25_setf(uint8_t swprof25)
{
    ASSERT_ERR((((uint32_t)swprof25 << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)swprof25 << 25));
}

__INLINE uint8_t ip_swprofiling_swprof24_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void ip_swprofiling_swprof24_setf(uint8_t swprof24)
{
    ASSERT_ERR((((uint32_t)swprof24 << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)swprof24 << 24));
}

__INLINE uint8_t ip_swprofiling_swprof23_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void ip_swprofiling_swprof23_setf(uint8_t swprof23)
{
    ASSERT_ERR((((uint32_t)swprof23 << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)swprof23 << 23));
}

__INLINE uint8_t ip_swprofiling_swprof22_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void ip_swprofiling_swprof22_setf(uint8_t swprof22)
{
    ASSERT_ERR((((uint32_t)swprof22 << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)swprof22 << 22));
}

__INLINE uint8_t ip_swprofiling_swprof21_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void ip_swprofiling_swprof21_setf(uint8_t swprof21)
{
    ASSERT_ERR((((uint32_t)swprof21 << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)swprof21 << 21));
}

__INLINE uint8_t ip_swprofiling_swprof20_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void ip_swprofiling_swprof20_setf(uint8_t swprof20)
{
    ASSERT_ERR((((uint32_t)swprof20 << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)swprof20 << 20));
}

__INLINE uint8_t ip_swprofiling_swprof19_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void ip_swprofiling_swprof19_setf(uint8_t swprof19)
{
    ASSERT_ERR((((uint32_t)swprof19 << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)swprof19 << 19));
}

__INLINE uint8_t ip_swprofiling_swprof18_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void ip_swprofiling_swprof18_setf(uint8_t swprof18)
{
    ASSERT_ERR((((uint32_t)swprof18 << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)swprof18 << 18));
}

__INLINE uint8_t ip_swprofiling_swprof17_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void ip_swprofiling_swprof17_setf(uint8_t swprof17)
{
    ASSERT_ERR((((uint32_t)swprof17 << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)swprof17 << 17));
}

__INLINE uint8_t ip_swprofiling_swprof16_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void ip_swprofiling_swprof16_setf(uint8_t swprof16)
{
    ASSERT_ERR((((uint32_t)swprof16 << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)swprof16 << 16));
}

__INLINE uint8_t ip_swprofiling_swprof15_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ip_swprofiling_swprof15_setf(uint8_t swprof15)
{
    ASSERT_ERR((((uint32_t)swprof15 << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)swprof15 << 15));
}

__INLINE uint8_t ip_swprofiling_swprof14_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ip_swprofiling_swprof14_setf(uint8_t swprof14)
{
    ASSERT_ERR((((uint32_t)swprof14 << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)swprof14 << 14));
}

__INLINE uint8_t ip_swprofiling_swprof13_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ip_swprofiling_swprof13_setf(uint8_t swprof13)
{
    ASSERT_ERR((((uint32_t)swprof13 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)swprof13 << 13));
}

__INLINE uint8_t ip_swprofiling_swprof12_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ip_swprofiling_swprof12_setf(uint8_t swprof12)
{
    ASSERT_ERR((((uint32_t)swprof12 << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)swprof12 << 12));
}

__INLINE uint8_t ip_swprofiling_swprof11_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ip_swprofiling_swprof11_setf(uint8_t swprof11)
{
    ASSERT_ERR((((uint32_t)swprof11 << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)swprof11 << 11));
}

__INLINE uint8_t ip_swprofiling_swprof10_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ip_swprofiling_swprof10_setf(uint8_t swprof10)
{
    ASSERT_ERR((((uint32_t)swprof10 << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)swprof10 << 10));
}

__INLINE uint8_t ip_swprofiling_swprof9_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ip_swprofiling_swprof9_setf(uint8_t swprof9)
{
    ASSERT_ERR((((uint32_t)swprof9 << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)swprof9 << 9));
}

__INLINE uint8_t ip_swprofiling_swprof8_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ip_swprofiling_swprof8_setf(uint8_t swprof8)
{
    ASSERT_ERR((((uint32_t)swprof8 << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)swprof8 << 8));
}

__INLINE uint8_t ip_swprofiling_swprof7_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ip_swprofiling_swprof7_setf(uint8_t swprof7)
{
    ASSERT_ERR((((uint32_t)swprof7 << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)swprof7 << 7));
}

__INLINE uint8_t ip_swprofiling_swprof6_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void ip_swprofiling_swprof6_setf(uint8_t swprof6)
{
    ASSERT_ERR((((uint32_t)swprof6 << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)swprof6 << 6));
}

__INLINE uint8_t ip_swprofiling_swprof5_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void ip_swprofiling_swprof5_setf(uint8_t swprof5)
{
    ASSERT_ERR((((uint32_t)swprof5 << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)swprof5 << 5));
}

__INLINE uint8_t ip_swprofiling_swprof4_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ip_swprofiling_swprof4_setf(uint8_t swprof4)
{
    ASSERT_ERR((((uint32_t)swprof4 << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)swprof4 << 4));
}

__INLINE uint8_t ip_swprofiling_swprof3_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ip_swprofiling_swprof3_setf(uint8_t swprof3)
{
    ASSERT_ERR((((uint32_t)swprof3 << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)swprof3 << 3));
}

__INLINE uint8_t ip_swprofiling_swprof2_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ip_swprofiling_swprof2_setf(uint8_t swprof2)
{
    ASSERT_ERR((((uint32_t)swprof2 << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)swprof2 << 2));
}

__INLINE uint8_t ip_swprofiling_swprof1_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ip_swprofiling_swprof1_setf(uint8_t swprof1)
{
    ASSERT_ERR((((uint32_t)swprof1 << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)swprof1 << 1));
}

__INLINE uint8_t ip_swprofiling_swprof0_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_SWPROFILING_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_swprofiling_swprof0_setf(uint8_t swprof0)
{
    ASSERT_ERR((((uint32_t)swprof0 << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_SWPROFILING_ADDR, (REG_IP_RD(IP_SWPROFILING_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)swprof0 << 0));
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
#define IP_RADIOCNTL0_ADDR   0x40100070
#define IP_RADIOCNTL0_OFFSET 0x00000070
#define IP_RADIOCNTL0_INDEX  0x0000001C
#define IP_RADIOCNTL0_RESET  0x00000002

__INLINE uint32_t ip_radiocntl0_get(void)
{
    return REG_IP_RD(IP_RADIOCNTL0_ADDR);
}

__INLINE void ip_radiocntl0_set(uint32_t value)
{
    REG_IP_WR(IP_RADIOCNTL0_ADDR, value);
}

// field definitions
#define IP_SPIPTR_MASK    ((uint32_t)0xFFFF0000)
#define IP_SPIPTR_LSB     16
#define IP_SPIPTR_WIDTH   ((uint32_t)0x00000010)
#define IP_SPICFG_BIT     ((uint32_t)0x00000080)
#define IP_SPICFG_POS     7
#define IP_SPIFREQ_MASK   ((uint32_t)0x00000030)
#define IP_SPIFREQ_LSB    4
#define IP_SPIFREQ_WIDTH  ((uint32_t)0x00000002)
#define IP_SPICOMP_BIT    ((uint32_t)0x00000002)
#define IP_SPICOMP_POS    1
#define IP_SPIGO_BIT      ((uint32_t)0x00000001)
#define IP_SPIGO_POS      0

#define IP_SPIPTR_RST     0x0
#define IP_SPICFG_RST     0x0
#define IP_SPIFREQ_RST    0x0
#define IP_SPICOMP_RST    0x1
#define IP_SPIGO_RST      0x0

__INLINE void ip_radiocntl0_pack(uint16_t spiptr, uint8_t spicfg, uint8_t spifreq, uint8_t spigo)
{
    ASSERT_ERR((((uint32_t)spiptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)spicfg << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)spifreq << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)spigo << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_RADIOCNTL0_ADDR, ((uint32_t)spiptr << 16) | ((uint32_t)spicfg << 7) | ((uint32_t)spifreq << 4) | ((uint32_t)spigo << 0));
}

__INLINE void ip_radiocntl0_unpack(uint16_t *spiptr, uint8_t *spicfg, uint8_t *spifreq, uint8_t *spicomp, uint8_t *spigo)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL0_ADDR);

    *spiptr = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *spicfg = (localVal & ((uint32_t)0x00000080)) >> 7;
    *spifreq = (localVal & ((uint32_t)0x00000030)) >> 4;
    *spicomp = (localVal & ((uint32_t)0x00000002)) >> 1;
    *spigo = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t ip_radiocntl0_spiptr_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ip_radiocntl0_spiptr_setf(uint16_t spiptr)
{
    ASSERT_ERR((((uint32_t)spiptr << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_IP_WR(IP_RADIOCNTL0_ADDR, (REG_IP_RD(IP_RADIOCNTL0_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)spiptr << 16));
}

__INLINE uint8_t ip_radiocntl0_spicfg_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void ip_radiocntl0_spicfg_setf(uint8_t spicfg)
{
    ASSERT_ERR((((uint32_t)spicfg << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_IP_WR(IP_RADIOCNTL0_ADDR, (REG_IP_RD(IP_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)spicfg << 7));
}

__INLINE uint8_t ip_radiocntl0_spifreq_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void ip_radiocntl0_spifreq_setf(uint8_t spifreq)
{
    ASSERT_ERR((((uint32_t)spifreq << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_IP_WR(IP_RADIOCNTL0_ADDR, (REG_IP_RD(IP_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)spifreq << 4));
}

__INLINE uint8_t ip_radiocntl0_spicomp_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ip_radiocntl0_spigo_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_radiocntl0_spigo_setf(uint8_t spigo)
{
    ASSERT_ERR((((uint32_t)spigo << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_RADIOCNTL0_ADDR, (REG_IP_RD(IP_RADIOCNTL0_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)spigo << 0));
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
#define IP_RADIOCNTL1_ADDR   0x40100074
#define IP_RADIOCNTL1_OFFSET 0x00000074
#define IP_RADIOCNTL1_INDEX  0x0000001D
#define IP_RADIOCNTL1_RESET  0x00000000

__INLINE uint32_t ip_radiocntl1_get(void)
{
    return REG_IP_RD(IP_RADIOCNTL1_ADDR);
}

__INLINE void ip_radiocntl1_set(uint32_t value)
{
    REG_IP_WR(IP_RADIOCNTL1_ADDR, value);
}

// field definitions
#define IP_FORCEAGC_EN_BIT        ((uint32_t)0x80000000)
#define IP_FORCEAGC_EN_POS        31
#define IP_FORCEIQ_BIT            ((uint32_t)0x40000000)
#define IP_FORCEIQ_POS            30
#define IP_RXDNSL_BIT             ((uint32_t)0x20000000)
#define IP_RXDNSL_POS             29
#define IP_TXDNSL_BIT             ((uint32_t)0x10000000)
#define IP_TXDNSL_POS             28
#define IP_FORCEAGC_LENGTH_MASK   ((uint32_t)0x0FFF0000)
#define IP_FORCEAGC_LENGTH_LSB    16
#define IP_FORCEAGC_LENGTH_WIDTH  ((uint32_t)0x0000000C)
#define IP_SYNC_PULSE_MODE_BIT    ((uint32_t)0x00008000)
#define IP_SYNC_PULSE_MODE_POS    15
#define IP_SYNC_PULSE_SRC_BIT     ((uint32_t)0x00004000)
#define IP_SYNC_PULSE_SRC_POS     14
#define IP_DPCORR_EN_BIT          ((uint32_t)0x00002000)
#define IP_DPCORR_EN_POS          13
#define IP_JEF_SELECT_BIT         ((uint32_t)0x00001000)
#define IP_JEF_SELECT_POS         12
#define IP_XRFSEL_MASK            ((uint32_t)0x000003F0)
#define IP_XRFSEL_LSB             4
#define IP_XRFSEL_WIDTH           ((uint32_t)0x00000006)
#define IP_SUBVERSION_MASK        ((uint32_t)0x0000000F)
#define IP_SUBVERSION_LSB         0
#define IP_SUBVERSION_WIDTH       ((uint32_t)0x00000004)

#define IP_FORCEAGC_EN_RST        0x0
#define IP_FORCEIQ_RST            0x0
#define IP_RXDNSL_RST             0x0
#define IP_TXDNSL_RST             0x0
#define IP_FORCEAGC_LENGTH_RST    0x0
#define IP_SYNC_PULSE_MODE_RST    0x0
#define IP_SYNC_PULSE_SRC_RST     0x0
#define IP_DPCORR_EN_RST          0x0
#define IP_JEF_SELECT_RST         0x0
#define IP_XRFSEL_RST             0x0
#define IP_SUBVERSION_RST         0x0

__INLINE void ip_radiocntl1_pack(uint8_t forceagcen, uint8_t forceiq, uint8_t rxdnsl, uint8_t txdnsl, uint16_t forceagclength, uint8_t syncpulsemode, uint8_t syncpulsesrc, uint8_t dpcorren, uint8_t jefselect, uint8_t xrfsel, uint8_t subversion)
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
    REG_IP_WR(IP_RADIOCNTL1_ADDR, ((uint32_t)forceagcen << 31) | ((uint32_t)forceiq << 30) | ((uint32_t)rxdnsl << 29) | ((uint32_t)txdnsl << 28) | ((uint32_t)forceagclength << 16) | ((uint32_t)syncpulsemode << 15) | ((uint32_t)syncpulsesrc << 14) | ((uint32_t)dpcorren << 13) | ((uint32_t)jefselect << 12) | ((uint32_t)xrfsel << 4) | ((uint32_t)subversion << 0));
}

__INLINE void ip_radiocntl1_unpack(uint8_t *forceagcen, uint8_t *forceiq, uint8_t *rxdnsl, uint8_t *txdnsl, uint16_t *forceagclength, uint8_t *syncpulsemode, uint8_t *syncpulsesrc, uint8_t *dpcorren, uint8_t *jefselect, uint8_t *xrfsel, uint8_t *subversion)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);

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

__INLINE uint8_t ip_radiocntl1_forceagc_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ip_radiocntl1_forceagc_en_setf(uint8_t forceagcen)
{
    ASSERT_ERR((((uint32_t)forceagcen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)forceagcen << 31));
}

__INLINE uint8_t ip_radiocntl1_forceiq_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ip_radiocntl1_forceiq_setf(uint8_t forceiq)
{
    ASSERT_ERR((((uint32_t)forceiq << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)forceiq << 30));
}

__INLINE uint8_t ip_radiocntl1_rxdnsl_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void ip_radiocntl1_rxdnsl_setf(uint8_t rxdnsl)
{
    ASSERT_ERR((((uint32_t)rxdnsl << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)rxdnsl << 29));
}

__INLINE uint8_t ip_radiocntl1_txdnsl_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void ip_radiocntl1_txdnsl_setf(uint8_t txdnsl)
{
    ASSERT_ERR((((uint32_t)txdnsl << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)txdnsl << 28));
}

__INLINE uint16_t ip_radiocntl1_forceagc_length_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void ip_radiocntl1_forceagc_length_setf(uint16_t forceagclength)
{
    ASSERT_ERR((((uint32_t)forceagclength << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)forceagclength << 16));
}

__INLINE uint8_t ip_radiocntl1_sync_pulse_mode_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void ip_radiocntl1_sync_pulse_mode_setf(uint8_t syncpulsemode)
{
    ASSERT_ERR((((uint32_t)syncpulsemode << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)syncpulsemode << 15));
}

__INLINE uint8_t ip_radiocntl1_sync_pulse_src_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void ip_radiocntl1_sync_pulse_src_setf(uint8_t syncpulsesrc)
{
    ASSERT_ERR((((uint32_t)syncpulsesrc << 14) & ~((uint32_t)0x00004000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)syncpulsesrc << 14));
}

__INLINE uint8_t ip_radiocntl1_dpcorr_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ip_radiocntl1_dpcorr_en_setf(uint8_t dpcorren)
{
    ASSERT_ERR((((uint32_t)dpcorren << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)dpcorren << 13));
}

__INLINE uint8_t ip_radiocntl1_jef_select_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ip_radiocntl1_jef_select_setf(uint8_t jefselect)
{
    ASSERT_ERR((((uint32_t)jefselect << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)jefselect << 12));
}

__INLINE uint8_t ip_radiocntl1_xrfsel_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000003F0)) >> 4);
}

__INLINE void ip_radiocntl1_xrfsel_setf(uint8_t xrfsel)
{
    ASSERT_ERR((((uint32_t)xrfsel << 4) & ~((uint32_t)0x000003F0)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x000003F0)) | ((uint32_t)xrfsel << 4));
}

__INLINE uint8_t ip_radiocntl1_subversion_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RADIOCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ip_radiocntl1_subversion_setf(uint8_t subversion)
{
    ASSERT_ERR((((uint32_t)subversion << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_IP_WR(IP_RADIOCNTL1_ADDR, (REG_IP_RD(IP_RADIOCNTL1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)subversion << 0));
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
#define IP_AESCNTL_ADDR   0x401000B0
#define IP_AESCNTL_OFFSET 0x000000B0
#define IP_AESCNTL_INDEX  0x0000002C
#define IP_AESCNTL_RESET  0x00000000

__INLINE uint32_t ip_aescntl_get(void)
{
    return REG_IP_RD(IP_AESCNTL_ADDR);
}

__INLINE void ip_aescntl_set(uint32_t value)
{
    REG_IP_WR(IP_AESCNTL_ADDR, value);
}

// field definitions
#define IP_AES_MODE_BIT     ((uint32_t)0x00000002)
#define IP_AES_MODE_POS     1
#define IP_AES_START_BIT    ((uint32_t)0x00000001)
#define IP_AES_START_POS    0

#define IP_AES_MODE_RST     0x0
#define IP_AES_START_RST    0x0

__INLINE void ip_aescntl_pack(uint8_t aesmode, uint8_t aesstart)
{
    ASSERT_ERR((((uint32_t)aesmode << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)aesstart << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_AESCNTL_ADDR, ((uint32_t)aesmode << 1) | ((uint32_t)aesstart << 0));
}

__INLINE void ip_aescntl_unpack(uint8_t *aesmode, uint8_t *aesstart)
{
    uint32_t localVal = REG_IP_RD(IP_AESCNTL_ADDR);

    *aesmode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *aesstart = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_aescntl_aes_mode_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_AESCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ip_aescntl_aes_mode_setf(uint8_t aesmode)
{
    ASSERT_ERR((((uint32_t)aesmode << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_IP_WR(IP_AESCNTL_ADDR, (REG_IP_RD(IP_AESCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)aesmode << 1));
}

__INLINE uint8_t ip_aescntl_aes_start_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_AESCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_aescntl_aes_start_setf(uint8_t aesstart)
{
    ASSERT_ERR((((uint32_t)aesstart << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_AESCNTL_ADDR, (REG_IP_RD(IP_AESCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)aesstart << 0));
}

/**
 * @brief AESKEY31_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           AESKEY31_0   0x0
 * </pre>
 */
#define IP_AESKEY31_0_ADDR   0x401000B4
#define IP_AESKEY31_0_OFFSET 0x000000B4
#define IP_AESKEY31_0_INDEX  0x0000002D
#define IP_AESKEY31_0_RESET  0x00000000

__INLINE uint32_t ip_aeskey31_0_get(void)
{
    return REG_IP_RD(IP_AESKEY31_0_ADDR);
}

__INLINE void ip_aeskey31_0_set(uint32_t value)
{
    REG_IP_WR(IP_AESKEY31_0_ADDR, value);
}

// field definitions
#define IP_AESKEY31_0_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_AESKEY31_0_LSB    0
#define IP_AESKEY31_0_WIDTH  ((uint32_t)0x00000020)

#define IP_AESKEY31_0_RST    0x0

__INLINE uint32_t ip_aeskey31_0_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_AESKEY31_0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_aeskey31_0_setf(uint32_t aeskey310)
{
    ASSERT_ERR((((uint32_t)aeskey310 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_AESKEY31_0_ADDR, (uint32_t)aeskey310 << 0);
}

/**
 * @brief AESKEY63_32 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          AESKEY63_32   0x0
 * </pre>
 */
#define IP_AESKEY63_32_ADDR   0x401000B8
#define IP_AESKEY63_32_OFFSET 0x000000B8
#define IP_AESKEY63_32_INDEX  0x0000002E
#define IP_AESKEY63_32_RESET  0x00000000

__INLINE uint32_t ip_aeskey63_32_get(void)
{
    return REG_IP_RD(IP_AESKEY63_32_ADDR);
}

__INLINE void ip_aeskey63_32_set(uint32_t value)
{
    REG_IP_WR(IP_AESKEY63_32_ADDR, value);
}

// field definitions
#define IP_AESKEY63_32_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_AESKEY63_32_LSB    0
#define IP_AESKEY63_32_WIDTH  ((uint32_t)0x00000020)

#define IP_AESKEY63_32_RST    0x0

__INLINE uint32_t ip_aeskey63_32_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_AESKEY63_32_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_aeskey63_32_setf(uint32_t aeskey6332)
{
    ASSERT_ERR((((uint32_t)aeskey6332 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_AESKEY63_32_ADDR, (uint32_t)aeskey6332 << 0);
}

/**
 * @brief AESKEY95_64 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          AESKEY95_64   0x0
 * </pre>
 */
#define IP_AESKEY95_64_ADDR   0x401000BC
#define IP_AESKEY95_64_OFFSET 0x000000BC
#define IP_AESKEY95_64_INDEX  0x0000002F
#define IP_AESKEY95_64_RESET  0x00000000

__INLINE uint32_t ip_aeskey95_64_get(void)
{
    return REG_IP_RD(IP_AESKEY95_64_ADDR);
}

__INLINE void ip_aeskey95_64_set(uint32_t value)
{
    REG_IP_WR(IP_AESKEY95_64_ADDR, value);
}

// field definitions
#define IP_AESKEY95_64_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_AESKEY95_64_LSB    0
#define IP_AESKEY95_64_WIDTH  ((uint32_t)0x00000020)

#define IP_AESKEY95_64_RST    0x0

__INLINE uint32_t ip_aeskey95_64_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_AESKEY95_64_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_aeskey95_64_setf(uint32_t aeskey9564)
{
    ASSERT_ERR((((uint32_t)aeskey9564 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_AESKEY95_64_ADDR, (uint32_t)aeskey9564 << 0);
}

/**
 * @brief AESKEY127_96 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         AESKEY127_96   0x0
 * </pre>
 */
#define IP_AESKEY127_96_ADDR   0x401000C0
#define IP_AESKEY127_96_OFFSET 0x000000C0
#define IP_AESKEY127_96_INDEX  0x00000030
#define IP_AESKEY127_96_RESET  0x00000000

__INLINE uint32_t ip_aeskey127_96_get(void)
{
    return REG_IP_RD(IP_AESKEY127_96_ADDR);
}

__INLINE void ip_aeskey127_96_set(uint32_t value)
{
    REG_IP_WR(IP_AESKEY127_96_ADDR, value);
}

// field definitions
#define IP_AESKEY127_96_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_AESKEY127_96_LSB    0
#define IP_AESKEY127_96_WIDTH  ((uint32_t)0x00000020)

#define IP_AESKEY127_96_RST    0x0

__INLINE uint32_t ip_aeskey127_96_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_AESKEY127_96_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_aeskey127_96_setf(uint32_t aeskey12796)
{
    ASSERT_ERR((((uint32_t)aeskey12796 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_IP_WR(IP_AESKEY127_96_ADDR, (uint32_t)aeskey12796 << 0);
}

/**
 * @brief AESPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               AESPTR   0x0
 * </pre>
 */
#define IP_AESPTR_ADDR   0x401000C4
#define IP_AESPTR_OFFSET 0x000000C4
#define IP_AESPTR_INDEX  0x00000031
#define IP_AESPTR_RESET  0x00000000

__INLINE uint32_t ip_aesptr_get(void)
{
    return REG_IP_RD(IP_AESPTR_ADDR);
}

__INLINE void ip_aesptr_set(uint32_t value)
{
    REG_IP_WR(IP_AESPTR_ADDR, value);
}

// field definitions
#define IP_AESPTR_MASK   ((uint32_t)0x0000FFFF)
#define IP_AESPTR_LSB    0
#define IP_AESPTR_WIDTH  ((uint32_t)0x00000010)

#define IP_AESPTR_RST    0x0

__INLINE uint16_t ip_aesptr_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_AESPTR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_aesptr_setf(uint16_t aesptr)
{
    ASSERT_ERR((((uint32_t)aesptr << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_IP_WR(IP_AESPTR_ADDR, (uint32_t)aesptr << 0);
}

/**
 * @brief TXMICVAL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             TXMICVAL   0x0
 * </pre>
 */
#define IP_TXMICVAL_ADDR   0x401000C8
#define IP_TXMICVAL_OFFSET 0x000000C8
#define IP_TXMICVAL_INDEX  0x00000032
#define IP_TXMICVAL_RESET  0x00000000

__INLINE uint32_t ip_txmicval_get(void)
{
    return REG_IP_RD(IP_TXMICVAL_ADDR);
}

// field definitions
#define IP_TXMICVAL_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_TXMICVAL_LSB    0
#define IP_TXMICVAL_WIDTH  ((uint32_t)0x00000020)

#define IP_TXMICVAL_RST    0x0

__INLINE uint32_t ip_txmicval_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_TXMICVAL_ADDR);
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
#define IP_RXMICVAL_ADDR   0x401000CC
#define IP_RXMICVAL_OFFSET 0x000000CC
#define IP_RXMICVAL_INDEX  0x00000033
#define IP_RXMICVAL_RESET  0x00000000

__INLINE uint32_t ip_rxmicval_get(void)
{
    return REG_IP_RD(IP_RXMICVAL_ADDR);
}

// field definitions
#define IP_RXMICVAL_MASK   ((uint32_t)0xFFFFFFFF)
#define IP_RXMICVAL_LSB    0
#define IP_RXMICVAL_WIDTH  ((uint32_t)0x00000020)

#define IP_RXMICVAL_RST    0x0

__INLINE uint32_t ip_rxmicval_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_RXMICVAL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief PRIOSCHARB register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16          BLEPRIOMODE   1
 *     00        BREDRPRIOMODE   1
 * </pre>
 */
#define IP_PRIOSCHARB_ADDR   0x401000D0
#define IP_PRIOSCHARB_OFFSET 0x000000D0
#define IP_PRIOSCHARB_INDEX  0x00000034
#define IP_PRIOSCHARB_RESET  0x00010001

__INLINE uint32_t ip_prioscharb_get(void)
{
    return REG_IP_RD(IP_PRIOSCHARB_ADDR);
}

__INLINE void ip_prioscharb_set(uint32_t value)
{
    REG_IP_WR(IP_PRIOSCHARB_ADDR, value);
}

// field definitions
#define IP_BLEPRIOMODE_BIT      ((uint32_t)0x00010000)
#define IP_BLEPRIOMODE_POS      16
#define IP_BREDRPRIOMODE_BIT    ((uint32_t)0x00000001)
#define IP_BREDRPRIOMODE_POS    0

#define IP_BLEPRIOMODE_RST      0x1
#define IP_BREDRPRIOMODE_RST    0x1

__INLINE void ip_prioscharb_pack(uint8_t blepriomode, uint8_t bredrpriomode)
{
    ASSERT_ERR((((uint32_t)blepriomode << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)bredrpriomode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_PRIOSCHARB_ADDR, ((uint32_t)blepriomode << 16) | ((uint32_t)bredrpriomode << 0));
}

__INLINE void ip_prioscharb_unpack(uint8_t *blepriomode, uint8_t *bredrpriomode)
{
    uint32_t localVal = REG_IP_RD(IP_PRIOSCHARB_ADDR);

    *blepriomode = (localVal & ((uint32_t)0x00010000)) >> 16;
    *bredrpriomode = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ip_prioscharb_blepriomode_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_PRIOSCHARB_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void ip_prioscharb_blepriomode_setf(uint8_t blepriomode)
{
    ASSERT_ERR((((uint32_t)blepriomode << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_IP_WR(IP_PRIOSCHARB_ADDR, (REG_IP_RD(IP_PRIOSCHARB_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)blepriomode << 16));
}

__INLINE uint8_t ip_prioscharb_bredrpriomode_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_PRIOSCHARB_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ip_prioscharb_bredrpriomode_setf(uint8_t bredrpriomode)
{
    ASSERT_ERR((((uint32_t)bredrpriomode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(IP_PRIOSCHARB_ADDR, (REG_IP_RD(IP_PRIOSCHARB_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)bredrpriomode << 0));
}

/**
 * @brief GROSSTIMTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  22:00          GROSSTARGET   0x0
 * </pre>
 */
#define IP_GROSSTIMTGT_ADDR   0x401000E4
#define IP_GROSSTIMTGT_OFFSET 0x000000E4
#define IP_GROSSTIMTGT_INDEX  0x00000039
#define IP_GROSSTIMTGT_RESET  0x00000000

__INLINE uint32_t ip_grosstimtgt_get(void)
{
    return REG_IP_RD(IP_GROSSTIMTGT_ADDR);
}

__INLINE void ip_grosstimtgt_set(uint32_t value)
{
    REG_IP_WR(IP_GROSSTIMTGT_ADDR, value);
}

// field definitions
#define IP_GROSSTARGET_MASK   ((uint32_t)0x007FFFFF)
#define IP_GROSSTARGET_LSB    0
#define IP_GROSSTARGET_WIDTH  ((uint32_t)0x00000017)

#define IP_GROSSTARGET_RST    0x0

__INLINE uint32_t ip_grosstimtgt_grosstarget_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_GROSSTIMTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x007FFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_grosstimtgt_grosstarget_setf(uint32_t grosstarget)
{
    ASSERT_ERR((((uint32_t)grosstarget << 0) & ~((uint32_t)0x007FFFFF)) == 0);
    REG_IP_WR(IP_GROSSTIMTGT_ADDR, (uint32_t)grosstarget << 0);
}

/**
 * @brief FINETIMTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00           FINETARGET   0x0
 * </pre>
 */
#define IP_FINETIMTGT_ADDR   0x401000E8
#define IP_FINETIMTGT_OFFSET 0x000000E8
#define IP_FINETIMTGT_INDEX  0x0000003A
#define IP_FINETIMTGT_RESET  0x00000000

__INLINE uint32_t ip_finetimtgt_get(void)
{
    return REG_IP_RD(IP_FINETIMTGT_ADDR);
}

__INLINE void ip_finetimtgt_set(uint32_t value)
{
    REG_IP_WR(IP_FINETIMTGT_ADDR, value);
}

// field definitions
#define IP_FINETARGET_MASK   ((uint32_t)0x0FFFFFFF)
#define IP_FINETARGET_LSB    0
#define IP_FINETARGET_WIDTH  ((uint32_t)0x0000001C)

#define IP_FINETARGET_RST    0x0

__INLINE uint32_t ip_finetimtgt_finetarget_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_FINETIMTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_finetimtgt_finetarget_setf(uint32_t finetarget)
{
    ASSERT_ERR((((uint32_t)finetarget << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_IP_WR(IP_FINETIMTGT_ADDR, (uint32_t)finetarget << 0);
}

/**
 * @brief CLKNTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:00              CLKNTGT   0x0
 * </pre>
 */
#define IP_CLKNTGT_ADDR   0x401000EC
#define IP_CLKNTGT_OFFSET 0x000000EC
#define IP_CLKNTGT_INDEX  0x0000003B
#define IP_CLKNTGT_RESET  0x00000000

__INLINE uint32_t ip_clkntgt_get(void)
{
    return REG_IP_RD(IP_CLKNTGT_ADDR);
}

__INLINE void ip_clkntgt_set(uint32_t value)
{
    REG_IP_WR(IP_CLKNTGT_ADDR, value);
}

// field definitions
#define IP_CLKNTGT_MASK   ((uint32_t)0x0FFFFFFF)
#define IP_CLKNTGT_LSB    0
#define IP_CLKNTGT_WIDTH  ((uint32_t)0x0000001C)

#define IP_CLKNTGT_RST    0x0

__INLINE uint32_t ip_clkntgt_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_CLKNTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0FFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_clkntgt_setf(uint32_t clkntgt)
{
    ASSERT_ERR((((uint32_t)clkntgt << 0) & ~((uint32_t)0x0FFFFFFF)) == 0);
    REG_IP_WR(IP_CLKNTGT_ADDR, (uint32_t)clkntgt << 0);
}

/**
 * @brief HMICROSECTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00         HMICROSECTGT   0x0
 * </pre>
 */
#define IP_HMICROSECTGT_ADDR   0x401000F0
#define IP_HMICROSECTGT_OFFSET 0x000000F0
#define IP_HMICROSECTGT_INDEX  0x0000003C
#define IP_HMICROSECTGT_RESET  0x00000000

__INLINE uint32_t ip_hmicrosectgt_get(void)
{
    return REG_IP_RD(IP_HMICROSECTGT_ADDR);
}

__INLINE void ip_hmicrosectgt_set(uint32_t value)
{
    REG_IP_WR(IP_HMICROSECTGT_ADDR, value);
}

// field definitions
#define IP_HMICROSECTGT_MASK   ((uint32_t)0x000003FF)
#define IP_HMICROSECTGT_LSB    0
#define IP_HMICROSECTGT_WIDTH  ((uint32_t)0x0000000A)

#define IP_HMICROSECTGT_RST    0x0

__INLINE uint16_t ip_hmicrosectgt_getf(void)
{
    uint32_t localVal = REG_IP_RD(IP_HMICROSECTGT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000003FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ip_hmicrosectgt_setf(uint16_t hmicrosectgt)
{
    ASSERT_ERR((((uint32_t)hmicrosectgt << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(IP_HMICROSECTGT_ADDR, (uint32_t)hmicrosectgt << 0);
}


#endif // _REG_IPCORE_H_

